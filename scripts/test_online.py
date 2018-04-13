from rostopics_to_timeseries.RostopicsToTimeseries import OnlineRostopicsToTimeseries
from rostopics_to_timeseries.TopicMsgToVector import TopicMsgToVector, BaxterEndpointStateToVector
import baxter_core_msgs.msg
import rospy

if __name__ == '__main__':
    rospy.init_node("test_RostopicsToTimeseries")
    topic_info = [
        (
            "/robot/limb/right/endpoint_state", 
            baxter_core_msgs.msg.EndpointState, 
            BaxterEndpointStateToVector,
        ),
    ] 
    onrt = OnlineRostopicsToTimeseries(topic_info, rate=10) 
    onrt.start_publishing_timeseries("/rostopics_to_timeseries_topic")
