# stefmap_ros
ROS implementation to call a STeF-Map prediction

## Guide to test the server:
1. Update both repositories
	- STeF-Map 
	- stefmap_ros

2. (*roscore*)

3. rosrun fremenarray fremenarray

4. [assuming that u have already created the histogram files. e.g. "atc-20121024-histograms.txt"]
	(*python <directory>/STeF-Map/load_2_days.py*)

5. (*rosrun stefmap_ros stefmap_server.py*)

6. python .../stefmap_ros/src/scripts/stefmap_client_example.py
	[u should see printed in the terminal a stemap with the structure specified in the msg folder with the files STeFMapMsg.msg and STeFMapCellMsg.msg]

