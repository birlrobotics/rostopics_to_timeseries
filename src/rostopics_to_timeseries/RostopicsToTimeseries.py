import rospy
import threading
from rostopics_to_timeseries.msg import Timeseries
import copy
import numpy as np
import std_msgs.msg
import rosbag
from scipy.interpolate import interp1d
class RostopicsToTimeseries(object):
    """
    Args:
        topic_filtering_config: A list of tuples telling topics and fields 
            that you're interested. Each tuple contains a string 
            of topic name, a class of message type and a subclass
            of TopicMsgFilter. Example:
                [
                    (
                        "/robot/limb/right/endpoint_state", 
                        baxter_core_msgs.msg.EndpointState, 
                        BaxterEndpointStateFilter,
                    ),
                    (
                        "/robotiq_force_torque_wrench",
                        geometry_msgs.msg.WrenchStamped,
                        WrenchStampedFilter,
                    ),
                ] 
        rate: Rate of time series in Hz.
    """

    def __init__(self, topic_filtering_config, rate):
        self.msg_filters = []
        self.topic_filtering_config = topic_filtering_config
        self.rate = rate

class OnlineRostopicsToTimeseries(RostopicsToTimeseries):
    def __init__(self, topic_filtering_config, rate):
        super(OnlineRostopicsToTimeseries, self).__init__(topic_filtering_config, rate)
        self.writable = threading.Event()
        self.writable.set()

    def _setup_listener(self):
        self.filtered_msgs = [None]*self.topic_filtering_config.get_filter_amount()
        self.subs = [None]*self.topic_filtering_config.get_filter_amount()
        def cb(data, arg):
            filter, filter_count = arg
            if self.writable.is_set():
                self.filtered_msgs[filter_count] = filter.convert(data)
            else:
                pass

        for filter_count, i in enumerate(self.topic_filtering_config.iter_filters()):
            topic_name, msg_type, filter = i
            self.subs[filter_count] = rospy.Subscriber(
                topic_name,
                msg_type,
                callback=cb,
                callback_args=(filter, filter_count),
            )
        
    def start_publishing_timeseries(self, topic_name):
        self._setup_listener()

        pub = rospy.Publisher(topic_name, Timeseries, queue_size=1000)

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.writable.clear()
            sample = copy.deepcopy(self.filtered_msgs)            
            self.writable.set()
            if len(sample) != 0:
                try:
                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time.now()
                    msg = Timeseries(h, np.concatenate(sample)) 
                    pub.publish(msg)
                except ValueError:
                    rospy.logerr("Cannot concatenate %s"%sample)
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                break

class OfflineRostopicsToTimeseries(RostopicsToTimeseries):
    def __init__(self, topic_filtering_config, rate):
        super(OfflineRostopicsToTimeseries, self).__init__(topic_filtering_config, rate)
        pass

    def get_timeseries_mat(self, path_to_rosbag):
        bag = rosbag.Bag(path_to_rosbag)        
        start_time = bag.get_start_time()
        end_time = bag.get_end_time()
        new_x = np.arange(start_time, end_time, 1.0/self.rate)

        mats = []
        for topic_name, msg_type, filter in self.topic_filtering_config.iter_filters():
            x = []
            mat = []
            for topic, msg, t, in bag.read_messages(topics=[topic_name]):
                x.append(t.to_sec())
                mat.append(filter.convert(msg))
        
            mat = np.array(mat)

            f = interp1d(x, mat, axis=0, kind='nearest', fill_value='extrapolate', assume_sorted=True)

            new_mat = f(new_x)

            mats.append(new_mat)

        big_mat = np.concatenate(mats, axis=1)
        return new_x, big_mat
