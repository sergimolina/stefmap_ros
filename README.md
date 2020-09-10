# stefmap_ros
STeF-map ROS implementation.
STeF-map is a time-dependent probabilistic map able to model and predict flow patterns of people in indoor environments. The proposed representation models the likelihood of motion direction on a grid-based map by a set of harmonic functions, which efficiently capture long-term variations of crowd movements over time [1].


There are two modalities to run the STeF-map: Online or Offline
- Online: the 
- Offile: this modality is useful when the data has been already gathered and we want to built the STeF-map manually (no need of active topics)

## Nodes:
### stefmap_node_online

#### Subscribed topics:

    /people_detections (geometry_msgs/PoseArray)
        Topic providing the people detections used later to build the histograms in each time interval and upda the STeF-map.

    /coverage_scan (sensor_msgs/LaserScan)
        Scan used to perform ray tracing and define the cells that are seen by the robot. Only the cells that can be seen are updated in each time interval. 
        This is done to differenciate between cells that are seen but there was no detections and cells that are not seen and hence no data was available.

#### Published topics:

    /visibility_map (nav_msgs/OccupancyGrid)
        Map showing the coverage applying ray tracing using the /coverage_scan topic.

    /stefmap (stefmap_ros/STeFMapMsg)
        This topic is published everytime the service ~get_stemap is called.

#### Services:
    ~get_stefmap (stefmap_ros/GetSTeFMap)
        It provides a STeF-map prediction given a time (unix) and the model order (frequency components to use to calculate the output)


#### Parameters:
    ~grid_size (float, default: 1)  
        Cell size. Recommended between 0.5m and 2m.

    ~x_min (float, default: -50)    
        Minimum x value of the STeF-map.

    ~x_max (float, default: 50)     
        Maximum x value of the STeF-map.

    ~y_min (float, default: -50)
        Minimum y value of the STeF-map.

    ~y_max (float, default: 50)
        Maximum y value of the STeF-map.

    ~interval_time (int, default: 600)
        Interval of time in seconds between STeF-map updates.

    ~num_bins (int, default: 8)
        Number of bins discretising the full circumference. Recommended 8 or 12.

    ~frame_id (string, default: map)
        The name of the STeF-map frame.

    ~people_detections_topic (string, default: people_detections)
        Name of the topic providing the people detections.

    ~coverage_laser_topic (string, default: coverage_scan)
        Name of the topic providing the coverage scan.
        
    ~coverage_time_update (float, default: 5)
        Frequency in seconds between coverage scan updates. The faster the robot can move the lower the value has to be.


### stefmap_node_offline
As mentioned before this way of building the STeF-map is useful when we arlready have the people dection data.
#### Subscribed topics:
    None
#### Published topics:
    /stefmap (stefmap_ros/STeFMapMsg)
        This topic is published everytime the service ~get_stemap is called.

#### Services:
    ~get_stefmap (stefmap_ros/GetSTeFMap)
        It provides a STeF-map prediction given a time (unix) and the model order (frequency components to use to calculate the output)
    
    ~update_stefmap (stefmap_ros/UpdateSTeFMap)
        This service allows to update the STeF-map with the data provided.
        

#### Parameters:
    ~grid_size (float, default: 1)  
        Cell size. Recommended between 0.5m and 2m.

    ~x_min (float, default: -50)    
        Minimum x value of the STeF-map.

    ~x_max (float, default: 50)     
        Maximum x value of the STeF-map.

    ~y_min (float, default: -50)
        Minimum y value of the STeF-map.

    ~y_max (float, default: 50)
        Maximum y value of the STeF-map.

    ~num_bins (int, default: 8)
        Number of bins discretising the full circumference. Recommended 8 or 12.

    ~frame_id (string, default: map)
        The name of the STeF-map frame.

## Contact
For further questions or doubts: smolinamellado@lincoln.ac.uk

## References
[1] S. Molina, G. Cielniak, T. Krajnik and T. Duckett. Modelling and Predicting Rhythmic Flow Patterns in Dyanmics Environments. In Towards  AutonomousRobotic Systems (TAROS), volume 10965, pages 135–146, 2018. 
