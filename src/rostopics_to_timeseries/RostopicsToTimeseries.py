import rospy
import threading
from rostopics_to_timeseries.msg import Timeseries
import copy
import numpy as np
import ipdb
import std_msgs.msg

TOPIC_NAME_IDX = 0
TOPIC_MSG_TYPE_IDX = 1
TOPIC_CB_IDX = 2

class RostopicsToTimeseries(object):
    """
    Args:
        topics_info: A list of tuples telling topics and fields 
            that you're interested. Each tuple contains a string 
            of topic name, a class of message type and a callback 
            function that filters each message. Example:
                [
                    (
                        "/robot/limb/right/endpoint_state", 
                        baxter_core_msgs.msg.EndpointState, 
                        lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z],
                    ),
                    (
                        "/robotiq_force_torque_wrench",
                        geometry_msgs.msg.WrenchStamped,
                        lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z],
                    ),
                ] 

    """

    def __init__(self, topics_info, rate):
        self.topics_info = topics_info
        self.rate = rate

class OnlineRostopicsToTimeseries(RostopicsToTimeseries):
    def __init__(self, topics_info, rate):
        super(OnlineRostopicsToTimeseries, self).__init__(topics_info, rate)
        self.writable = threading.Event()
        self.writable.set()

    def _setup_listener(self):
        self.filtered_msgs = [None]*len(self.topics_info)
        self.subs = [None]*len(self.topics_info)
        def cb_gen(TOPIC_IDX):
            def cb(data):
                if self.writable.is_set():
                    self.filtered_msgs[TOPIC_IDX] = self.topics_info[TOPIC_IDX][TOPIC_CB_IDX](data)
                else:
                    pass
            return cb
        for TOPIC_IDX, tu in enumerate(self.topics_info):
            self.subs[TOPIC_IDX] = rospy.Subscriber(
                tu[TOPIC_NAME_IDX],
                tu[TOPIC_MSG_TYPE_IDX],
                cb_gen(TOPIC_IDX)
            )
        rospy.sleep(1)
        
    def start_publishing_timeseries(self, topic_name):
        self._setup_listener()

        pub = rospy.Publisher(topic_name, Timeseries, queue_size=1000)

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.writable.clear()
            sample = copy.deepcopy(self.filtered_msgs)            
            self.writable.set()
            try:
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                msg = Timeseries(h, np.concatenate(sample)) 
                pub.publish(msg)
            except ValueError:
                rospy.logerr("Cannot concatenate %s"%sample)
            r.sleep()

class OfflineRostopicsToTimeseries(RostopicsToTimeseries):
    def __init__(self, topics_info, rate, path_to_rosbag):
        super(OfflineRostopicsToTimeseries, self).__init__(topics_info, rate)
        pass




