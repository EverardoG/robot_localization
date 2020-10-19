#!/usr/bin/env python3

""" This is the code for cropping out the first 20 seconds of the ac109_1 rosbag """
# This was ultimiately not a necessary tool, but could be useful for examples later on
import rosbag

cutoff_time = int(20.0e9) # 18 seconds

with rosbag.Bag('../bags/ac109_1.bag', 'w') as outbag:
    first_message = True
    for topic, msg, t in rosbag.Bag('../bags/ac109_1_original.bag').read_messages():

        # Just get the start time if this is the first message
        if first_message:
            start_time = t.to_nsec() # nanoseconds, one billionth of a second
            first_message = False

        # Only record a message if it happened after the specified interval
        # help(t)
        elif t.to_nsec() - start_time > cutoff_time:
            outbag.write(topic, msg, t)
