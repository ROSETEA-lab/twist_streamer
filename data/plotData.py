#!/usr/bin/env python3

import sys
import rosbag2_py
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import Twist


# Variables
bag_time_robot_cmdvel = []
robot_linvel = []
robot_angvel = []


# Open the bag
storage_options = rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr',output_serialization_format='cdr')
        
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)
        
topic_types = reader.get_all_topics_and_types()
        
# Create a map for quicker lookup
type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        
# Sequentially read the bag
t0 = -1

while reader.has_next():
	(topic, data, t) = reader.read_next()
	msg_type = get_message(type_map[topic])
	msg = deserialize_message(data, msg_type)
	
	if (t0<0):
		t0 = t
	
	# Extract cmd_vel
	if isinstance(msg, Twist) and (topic=='/cmd_vel'):
        	# Extract data from message and store it in temporary arrays
                bag_time_robot_cmdvel.append((t-t0)*1e-9)
                
                robot_linvel.append(msg.linear.x)
                robot_angvel.append(msg.angular.z)
                
# Plot data
plt.figure(1)
plt.subplot(211)
plt.plot(bag_time_robot_cmdvel,robot_linvel)
plt.xlabel("Time [s]")
plt.ylabel("Linear velocity [m/s]")
plt.subplot(212)
plt.plot(bag_time_robot_cmdvel,robot_angvel)
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")

plt.show()

