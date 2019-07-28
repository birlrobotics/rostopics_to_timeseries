#!/usr/bin/env python


from rostopics_to_timeseries import (
    RosTopicFilteringScheme, 
    TopicMsgFilter,
    OnlineRostopicsToTimeseries,
)
from rostopics_to_timeseries.TopicMsgFilter import BaxterEndpointStateFilter, BaxterEndpointStateFilterForTwistLinear

import baxter_core_msgs.msg
import rospy
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
from scipy import signal
import time

if __name__ == '__main__':
    rospy.init_node("test_RostopicsToTimeseries_node")
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

    tfc.smoother_class = WindowBasedSmoother_factory(signal.triang(5))

    onrt = OnlineRostopicsToTimeseries(tfc) 

    print 'test non blocking'
    onrt.start_publishing_timeseries("/rostopics_to_timeseries_topic", blocking=False)
    print 'should be publishing now. check rostopic echo'
    print 'hit enter to stop'
    raw_input()
    onrt.stop_publishing_timeseries()
    print 'should stop publishing now. check rostopic echo'
    print 'hit enter to start testing blocking'
    raw_input()
    onrt.start_publishing_timeseries("/rostopics_to_timeseries_topic")
