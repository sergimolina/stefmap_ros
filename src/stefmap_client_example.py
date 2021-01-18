#!/usr/bin/env python

import sys
import rospy
from stefmap_ros.srv import GetSTeFMap
from std_msgs.msg import Header
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.dom import minidom
from xml.etree import ElementTree

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


if __name__ == "__main__":

    prediction_order = 10
    prediction_time_ini = 3600*12
    prediction_time_end = 3600*12
    prediction_time_step = 3600
    loop = False
    save_in_xml_cliffmapformat = True
    output_cliffmapformat_filename = "orkla_stefmap_12pm_order10.xml"


    rospy.init_node("stefmap_client_node")
    rospy.sleep(1)

    prediction_time = 3600*12
    rospy.wait_for_service('get_stefmap')
    goOn = "True"
    while (not rospy.is_shutdown()) and (goOn == "True"):
        for t in range(prediction_time_ini,prediction_time_end+prediction_time_step,prediction_time_step):
            try:
                print "TIME: "+str(t/3600)+":00h"
                get_stefmap = rospy.ServiceProxy('get_stefmap', GetSTeFMap)
                stefmap = get_stefmap(t,prediction_order)

                if save_in_xml_cliffmapformat:
                    top = Element('map')

                    parameters = SubElement(top,'parameters')
                    x_min = SubElement(parameters,'x_min')
                    x_min.text = str(stefmap.stefmap.x_min)
                    x_max = SubElement(parameters,'x_max')
                    x_max.text = str(stefmap.stefmap.x_max)
                    y_min = SubElement(parameters,'y_min')
                    y_min.text = str(stefmap.stefmap.y_min)
                    y_max = SubElement(parameters,'y_max')
                    y_max.text = str(stefmap.stefmap.y_max)
                    step = SubElement(parameters,'step')
                    step.text = str(stefmap.stefmap.cell_size)
                    radious = SubElement(parameters,'radious')
                    radious.text = str(1)
                    wind = SubElement(parameters,'wind')
                    wind.text = str(0)
                    
                    locations = SubElement(top,'locations')
                    for c in range(1,len(stefmap.stefmap.cells)):
                        if stefmap.stefmap.cells[c].x >=0  and stefmap.stefmap.cells[c].y >=0:

                            location = SubElement(locations,'location')

                            id_ = SubElement(location,'id')
                            id_.text = str(c)
                            p = SubElement(location,'p')
                            q = SubElement(location,'q')
                            pose = SubElement(location,'pose')
                            x = SubElement(pose,'x')
                            x.text = str(stefmap.stefmap.cells[c].x)
                            y = SubElement(pose,'y')
                            y.text = str(stefmap.stefmap.cells[c].y)

                            total_sum_prob = sum(stefmap.stefmap.cells[c].probabilities[:])
                            if total_sum_prob > 0:
                                for b in range(0,8):
                                    if stefmap.stefmap.cells[c].probabilities[b] > 0:
                                        distribution = SubElement(location,'distribution')
                                        Cluster = SubElement(distribution,'Cluster')
                                        Cluster.text = str(-1)
                                        P = SubElement(distribution,'P')
                                        P.text = str(stefmap.stefmap.cells[c].probabilities[b]/total_sum_prob)
                                        M = SubElement(distribution,'M')
                                        th = SubElement(M,'th')
                                        th.text = str(b*0.785398)
                                        r = SubElement(M,'r')
                                        r.text = str(1)#3*stefmap.stefmap.cells[c].probabilities[b]/100)
                                        Cov = SubElement(distribution,'Cov')
                                        e_11 = SubElement(Cov,'e_11')
                                        e_11.text = str(0)
                                        e_12 = SubElement(Cov,'e_12')
                                        e_12.text = str(0)
                                        e_21 = SubElement(Cov,'e_21')
                                        e_21.text = str(0)
                                        e_22 = SubElement(Cov,'e_22')
                                        e_22.text = str(0)

                    top_pretty = prettify(top)
                    with open(output_cliffmapformat_filename,"w") as ofile:
                        ofile.write(top_pretty)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e 

            rospy.sleep(1)

        goOn = loop
