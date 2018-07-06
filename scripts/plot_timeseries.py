#!/usr/bin/env python

import rospy
from rostopics_to_timeseries.msg import Timeseries
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import copy
import ipdb

writable = threading.Event()
writable.set()

ms = []
dimension = None

def f1():
    def cb(m):
        global dimension
        if dimension is None:
            dimension = len(m.sample)
        if writable.is_set():
            ms.append(m)

    sub = rospy.Subscriber(
        "/rostopics_to_timeseries_topic",
        Timeseries,
        cb     
    )
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("plot_timeseries")
    t1 = threading.Thread(target=f1)
    t1.daemon = True
    t1.start()

    while not rospy.is_shutdown() and dimension is None:
        rospy.loginfo("Waiting for timeseries to be published")
        rospy.sleep(1)

    if not rospy.is_shutdown():
        fig, axs = plt.subplots(nrows=dimension, ncols=1)

        if dimension == 1:
            axs = [axs]

        window_size_in_sec = 10

        xdata = []
        ydatas = [[] for i in range(dimension)]
        yran = [0, 0]
        def update(frame):
            global ms, xdata, ydata, yran
            writable.clear()
            samples = copy.deepcopy(ms)
            ms = []
            writable.set()
        
            new_xs = [i.header.stamp.to_sec() for i in samples]
            xdata += new_xs
            cut_time = xdata[-1]-window_size_in_sec
            for idx, item in enumerate(xdata):
                if item >= cut_time:
                    cut_idx = idx
                    break
            xdata = xdata[cut_idx:]

            for dim_idx in range(dimension):
                new_ys = [i.sample[dim_idx] for i in samples]

                ydata = ydatas[dim_idx] 
                ydata += new_ys
                ydata = ydata[cut_idx:]
                ydatas[dim_idx] = ydata

                ax = axs[dim_idx]
                ax.set_xlim(xdata[0], xdata[-1])
                yran = [min(yran[0], min(ydata)), max(yran[1], max(ydata))]
                ax.set_ylim(yran[0]-1, yran[1]+1)
                ax.plot(xdata, ydata, 'r')

        ani = FuncAnimation(fig, update, interval=100)
        try:
            fig.show()
        except:
            pass
        rospy.spin()
