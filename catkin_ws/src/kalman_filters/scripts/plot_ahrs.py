#!/usr/bin/env python

from kalman_filters.msg import ahrs
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import sys
import rosbag
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Display AHRS roll and pitch angles from bag file')
    parser.add_argument('bag_file', type=str, help='full path to bag file to read data from')
    args = parser.parse_args()

    print("Loading bag file: {}".format(args.bag_file))

    roll_arr = []
    pitch_arr = []
    ts_arr = []
    
    bag = rosbag.Bag(args.bag_file)
    for topic, msg, t in bag.read_messages(topics=['/ahrs/pose']):
        roll_arr.append(msg.roll)
        pitch_arr.append(msg.pitch)
        ts_arr.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
    
    roll_np = np.array(roll_arr)
    pitch_np = np.array(pitch_arr)
    ts_np = np.array(ts_arr)

    fig, ax = plt.subplots()
    roll_line, = ax.plot(ts_np-ts_np[0], roll_np, label='Roll')
    pitch_line, = ax.plot(ts_np-ts_np[0], pitch_np, label='Pitch')
    ax.legend(handles=[roll_line, pitch_line])
    ax.set(xlabel='Time (s)', ylabel='Attitude Angle ($^\circ$)', title='AHRS Pitch and Roll Output')
    ax.grid()
    plt.show()