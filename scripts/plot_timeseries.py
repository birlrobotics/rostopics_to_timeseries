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

    while dimension is None:
        rospy.sleep(1)

    fig, axs = plt.subplots(nrows=dimension, ncols=1)

    if dimension == 1:
        axs = [axs]

    lns = [i.plot([], [], 'ro', animated=True)[0] for i in axs]

    window_size_in_sec = 10

    xdata = []
    ydatas = [[] for i in range(dimension)]
    xran = [None, None]
    yran = [None, None]
    def update(frame):
        global ms, xdata, ydata
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
            ax.set_ylim(min(ydata), max(ydata))

            ln = lns[dim_idx]
            ln.set_data(xdata, ydata)
              
        return lns

    ani = FuncAnimation(fig, update, interval=100, blit=True)
    plt.show()
