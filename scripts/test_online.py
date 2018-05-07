#!/usr/bin/env python

from rostopics_to_timeseries.RostopicsToTimeseries import OnlineRostopicsToTimeseries
from rostopics_to_timeseries.TopicMsgFilter import TopicMsgFilter, BaxterEndpointStateFilter
from rostopics_to_timeseries.RosTopicFilteringScheme import RosTopicFilteringScheme
import baxter_core_msgs.msg
import rospy
from rostopics_to_timeseries.Smoother import WindowBasedSmoother_factory
from scipy import signal

if __name__ == '__main__':
    rospy.init_node("test_RostopicsToTimeseries")
    tfc = RosTopicFilteringScheme(resampling_rate=10)
    tfc.add_filter(
        "/robot/limb/right/endpoint_state", 
        baxter_core_msgs.msg.EndpointState, 
        BaxterEndpointStateFilter,
    )

    tfc.smoother_class = WindowBasedSmoother_factory(signal.triang(51))

    onrt = OnlineRostopicsToTimeseries(tfc) 
    onrt.start_publishing_timeseries("/rostopics_to_timeseries_topic")
