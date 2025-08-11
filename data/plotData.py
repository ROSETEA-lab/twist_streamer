#!/usr/bin/env python3

import csv
import sys
import rosbag2_py
import matplotlib.pyplot as plt

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import Twist


# Variables
bag_time = []
bag_linvel = []
bag_angvel = []
csv_time = []
csv_linvel = []
csv_angvel = []

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
	
    # Extract cmd_vel
    if isinstance(msg, Twist) and (topic=='/cmd_vel'):
        # Take the first time instant
        if (t0<0):
            t0 = t

        # Extract data from message and store it in temporary arrays
        bag_time.append((t-t0)*1e-9)
        bag_linvel.append(msg.linear.x)
        bag_angvel.append(msg.angular.z)

# Open the CSV file
with open(sys.argv[2]) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
    	csv_time.append(float(row[0]))
    	csv_linvel.append(float(row[1]))
    	csv_angvel.append(float(row[2]))

    
# Plot data
plt.figure(1)
plt.subplot(211)
plt.plot(bag_time,bag_linvel,'b-', csv_time,csv_linvel,'r--')
plt.legend(['bag', 'csv'])
plt.xlabel("Time [s]")
plt.ylabel("Linear velocity [m/s]")
plt.subplot(212)
plt.plot(bag_time,bag_angvel,'b-', csv_time,csv_angvel,'r--')
plt.legend(['bag', 'csv'])
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")

plt.show()

