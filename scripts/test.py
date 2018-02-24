from rostopics_to_timeseries.RostopicsToTimeseries import OnlineRostopicsToTimeseries
import baxter_core_msgs.msg
import geometry_msgs.msg
import rospy

if __name__ == '__main__':
    rospy.init_node("test_RostopicsToTimeseries")
    topic_info = [
        (
            "/robot/limb/right/endpoint_state", 
            baxter_core_msgs.msg.EndpointState, 
            lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z],
        ),
        (
            "/robot/limb/right/endpoint_state",
            geometry_msgs.msg.WrenchStamped,
            lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z],
        ),
    ] 
    ort = OnlineRostopicsToTimeseries(topic_info, rate=100) 
    ort.start_publishing()
