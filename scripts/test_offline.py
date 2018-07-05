#!/usr/bin/env python

from rostopics_to_timeseries import (
    RosTopicFilteringScheme, 
    TopicMsgFilter,
    OfflineRostopicsToTimeseries, 
)

from rostopics_to_timeseries.TopicMsgFilter import BaxterEndpointStateFilter, BaxterEndpointStateFilterForTwistLinear
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory

import baxter_core_msgs.msg
import rospy
import matplotlib.pyplot as plt
from scipy import signal
import os

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))

if __name__ == '__main__':
    tfc = RosTopicFilteringScheme(resampling_rate=10)
    tfc.add_filter(
        "/robot/limb/right/endpoint_state", 
        baxter_core_msgs.msg.EndpointState, 
        BaxterEndpointStateFilter,
    )
    tfc.add_filter(
        "/robot/limb/right/endpoint_state", 
        baxter_core_msgs.msg.EndpointState, 
        BaxterEndpointStateFilterForTwistLinear,
    )

    tfc.smoother_class = WindowBasedSmoother_factory(signal.triang(51))
    tfc.smoother_class = None

    print 'timeseries_header:', tfc.timeseries_header
    print 'timeseries_size:', tfc.timeseries_size
    print 'filter_amount:', tfc.filter_amount
    print 'info:', tfc.info

    ofrt = OfflineRostopicsToTimeseries(tfc) 
    t, mat = ofrt.get_timeseries_mat(os.path.join(dir_of_this_script, "test_offline.bag"))

    dimension = mat.shape[1] 
    fig, axs = plt.subplots(nrows=dimension, ncols=1)
    if dimension == 1:
        axs = [axs]

    for dim_idx in range(dimension):
        ax = axs[dim_idx]
        x = t
        y = mat[:, dim_idx]
        ax.plot(x, y, 'ro')
        ax.set_title("index %s: %s"%(dim_idx, tfc.timeseries_header[dim_idx]))

    plt.show()
