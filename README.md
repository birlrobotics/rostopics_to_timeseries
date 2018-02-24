# Package: rostopics_to_timeseries

# Why This Package Exists

We want to have a tool that combines data from multiple ROS topics into a synchronized time series in a flexibly configurable way. This package attempts to do that.

# What Can This Package Do

- Online:
  
  Listen to multiple ROS topics, merge their messages into a fix-size vector, then publish this vector via a user-defined topic.

- Offline:

  Process topics in a rosbag file and extract a time series matrix from it.
  
# How To Use This Package 

Two parameters are needed from user:

- A list containing topic configuration:

  Users need to provide information about what topics to listen to, what their message types are, and how to process their messages into vectors.

  This information is received via a python list of tuples. Each tuple contains a topic name, the message type of that topic, and a function that transforms its message into a vector.

  An example would be:

    ```python
    [
        (
            "/robot/limb/right/endpoint_state", 
            baxter_core_msgs.msg.EndpointState, 
            lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z],
        ),
        (
            "/robotiq_force_torque_wrench",
            geometry_msgs.msg.WrenchStamped,
            lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z],
        ),
    ]
    ```
- An integer that specifies the rate of timeseries in Hz

# Demo
- Online
An online demo can be run via:
```bash
[In a new terminal]
cd ./scripts
python test_online.py

[In a new terminal]
cd ./scripts
python plot_timeseries.py
```

- Offline
An offline demo can be run via:
```bash
[In a new terminal]
cd ./scripts
python test_offline.py
```

# Minimal Tutorial

- Online

```python
from rostopics_to_timeseries.RostopicsToTimeseries import OnlineRostopicsToTimeseries
import baxter_core_msgs.msg
import geometry_msgs.msg
import rospy

if __name__ == '__main__':
    rospy.init_node("test_RostopicsToTimeseries") # Needed to init a ROS node first
    
    # Assemble a topic configuration
    topic_info = [ 
        (
            "/robot/limb/right/endpoint_state",
            baxter_core_msgs.msg.EndpointState,
            lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z],
        ),
        (
            "/robot/limb/right/endpoint_state",
            geometry_msgs.msg.WrenchStamped,
            lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z],
        ),
    ]
   
    # Pass in topic configuration and timeseries rate
    onrt = OnlineRostopicsToTimeseries(topic_info, rate=10) 
    
    # Pass in the topic name that publishes the timeseries vector
    onrt.start_publishing_timeseries("/rostopics_to_timeseries_topic") 
```

- Offline

```python
from rostopics_to_timeseries.RostopicsToTimeseries import OfflineRostopicsToTimeseries
import baxter_core_msgs.msg                                                         
import geometry_msgs.msg       
import rospy
import matplotlib.pyplot as plt
                          
if __name__ == '__main__':      

    # Assemble a topic configuration
    topic_info = [
        (
            "/robot/limb/right/endpoint_state",
            baxter_core_msgs.msg.EndpointState,
            lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z],
        ),
        (
            "/robot/limb/right/endpoint_state",
            geometry_msgs.msg.WrenchStamped,
            lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z],
        ),
    ]
    
    # Pass in topic configuration and timeseries rate
    ofrt = OfflineRostopicsToTimeseries(topic_info, rate=10)
    
    # Returns a list of time stamps and the timeseries matrix
    t, mat = ofrt.get_timeseries_mat("test_offline.bag")              
                                
    # Visualize the timeseries matrix 
    dimension = mat.shape[1]                                                                              
    fig, axs = plt.subplots(nrows=dimension, ncols=1)                                                     
    if dimension == 1:                                                                                    
        axs = [axs]                                                                                       
                                                                                                          
    for dim_idx in range(dimension):                                                                      
        ax = axs[dim_idx]                                                                                 
        x = t                                                                                             
        y = mat[:, dim_idx]                                                                               
        ax.plot(x, y, 'ro')                                                                               
                                                                                                          
    plt.show()
```
