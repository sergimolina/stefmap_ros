#!/usr/bin/env python

from stefmap_ros.msg import STeFMapMsg,STeFMapCellMsg
from stefmap_ros.srv import GetSTeFMap
import rospy
import numpy as np
from fremenarray.msg import FremenArrayActionGoal,FremenArrayActionResult
from os.path import expanduser
import time as t

predicted_probabilities = []

def result_fremen_callback(data):
	global predicted_probabilities
	if len(data.result.probabilities)>0:
		for i in range(0,len(data.result.probabilities)):
			predicted_probabilities.append(data.result.probabilities[i])
	print("Values received.")

def predict_map(timestamp,order):
	print("Making prediction...")
	pub = rospy.Publisher('/fremenarray/goal', FremenArrayActionGoal, queue_size=1)
	t.sleep(1)
	frem_msg=FremenArrayActionGoal()
	frem_msg.goal.operation = 'predict'
	frem_msg.goal.order = int(order)
	frem_msg.goal.time = timestamp
	pub.publish(frem_msg)
	t.sleep(1)


def handle_GetSTeFMap(req):
	global predicted_probabilities

	predict_map(req.prediction_time,req.order)

	rows = int((req.y_max - req.y_min)/req.cell_size)
	columns = int((req.x_max - req.x_min)/req.cell_size)
	
	prob_matrix = np.zeros((rows,columns,8))


	while len(predicted_probabilities) == 0:
		print len(predicted_probabilities)
		print("waiting...")
		t.sleep(0.1) 

	prob_matrix[:,:,:] = np.reshape(predicted_probabilities,(rows,columns,8))
	predicted_probabilities = []

	# Creating the ros msg to has to be returned to client calling
	STefMap = STeFMapMsg()

	STefMap.prediction_time = req.prediction_time	
	STefMap.x_min = req.x_min
	STefMap.x_max = req.x_max
	STefMap.y_min = req.y_min
	STefMap.y_max = req.y_max
	STefMap.cell_size = req.cell_size
	STefMap.rows = rows
	STefMap.columns = columns
	

	# iterate through all the cell and get also the bin with the maximum probability and the associated angle
	index = 0
	for r in range(0,rows):
		for c in range(0,columns):
			stefmap_cell = STeFMapCellMsg()

			stefmap_cell.row = int(r)
			stefmap_cell.column = int(c)
			stefmap_cell.x = float(req.x_min + req.cell_size*0.5 + req.cell_size * c)
			stefmap_cell.y = float(req.y_max - req.cell_size*0.5 - req.cell_size * r)
			stefmap_cell.probabilities = [float(prob_matrix[r,c,0]) ,
										  float(prob_matrix[r,c,1]) ,
										  float(prob_matrix[r,c,2]) ,
										  float(prob_matrix[r,c,3]) ,
										  float(prob_matrix[r,c,4]) ,
										  float(prob_matrix[r,c,5]) ,
										  float(prob_matrix[r,c,6]) ,
										  float(prob_matrix[r,c,7]) ] 

			max_number = -1
			for b in range(0,8):
				if prob_matrix[r,c,b] > max_number:
					max_number = prob_matrix[r,c,b]
					max_orientation = b

			stefmap_cell.best_angle = max_orientation*45

			STefMap.cell.append(stefmap_cell)
			index = index + 1

	print("STeFMap sent!")
	return STefMap

if __name__=="__main__":

	rospy.init_node('get_stefmap_server')
	rospy.Subscriber('/fremenarray/result',FremenArrayActionResult,result_fremen_callback)
	stefmap_service = rospy.Service('get_stefmap',GetSTeFMap,handle_GetSTeFMap)

	print "Ready to provide STeF-Maps!"
	rospy.spin()