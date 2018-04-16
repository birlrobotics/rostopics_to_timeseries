from rostopics_to_timeseries.RostopicsToTimeseries import OnlineRostopicsToTimeseries
from rostopics_to_timeseries.TopicMsgFilter import TopicMsgFilter, BaxterEndpointStateFilter
from rostopics_to_timeseries.RosTopicFilteringConfig import RosTopicFilteringConfig
import baxter_core_msgs.msg
import rospy

if __name__ == '__main__':
    rospy.init_node("test_RostopicsToTimeseries")
    tfc = RosTopicFilteringConfig()
    tfc.add_filter(
        "/robot/limb/right/endpoint_state", 
        baxter_core_msgs.msg.EndpointState, 
        BaxterEndpointStateFilter(),
    )
    onrt = OnlineRostopicsToTimeseries(tfc, rate=10) 
    onrt.start_publishing_timeseries("/rostopics_to_timeseries_topic")
