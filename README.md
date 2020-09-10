# stefmap_ros
STeF-map ROS implementation 

There are two modalities to run the STeF-map: Online or Offline
- Online: the 
- Offile: this modality is useful when the data has been already gathered and when we want to built the STeF-map manually (no need of active topics)

## Nodes:
### stefmap_node_online
#### Subscribed topics:

/people_detections (geometry_msgs/PoseArray)

/coverage_scan (sensor_msgs/LaserScan)

#### Published topics:

/visibility_map (nav_msgs/OccupancyGrid)

/stefmap (stefmap_ros/STeFMapMsg)

#### Services:
~get_stefmap (stefmap_ros/GetSTeFMap)


#### Parameters:
~grid_size (float, default: 1)
  Cell size. Recommended between 0.5m and 2m

~x_min (string, default: base_link)

~x_max (string, default: base_link)

~y_min (string, default: base_link)

~y_max (string, default: base_link)

~interval_time (string, default: base_link)

~num_bins (string, default: base_link)

~frame_id (string, default: base_link)

~people_detections_topic (string, default: base_link)

~coverage_laser_topic (string, default: base_link)

~coverage_time_update (string, default: base_link)




### stefmap_node_offline
As mentioned before this way of building the STeF-map is useful when we arlready have the people dection data.

