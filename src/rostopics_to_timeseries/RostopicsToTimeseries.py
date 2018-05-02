import rospy
import threading
from rostopics_to_timeseries.msg import Timeseries
import copy
import numpy as np
import std_msgs.msg
import rosbag
from scipy.interpolate import interp1d
import bisect
import ipdb

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

    def _setup_listener(self):
        self.raw_msgs = []
        self.filter_idx_to_msg_idx = []
        self.subs = []
        def cb(msg, callback_args):
            msg_idx = callback_args
            self.raw_msgs[msg_idx] = msg

        topic_to_msg_idx = {}
        for filter_count, (topic_name, msg_type, filter_class) in enumerate(self.topic_filtering_config.iter_filters()):
            if topic_name not in topic_to_msg_idx:
                self.raw_msgs.append(None)
                topic_to_msg_idx[topic_name] = len(self.raw_msgs)-1

                self.subs.append(rospy.Subscriber(
                    topic_name,
                    msg_type,
                    callback=cb,
                    callback_args=topic_to_msg_idx[topic_name],
                ))

            self.filter_idx_to_msg_idx.append(topic_to_msg_idx[topic_name])
        
    def start_publishing_timeseries(self, topic_name):
        self._setup_listener()

        pub = rospy.Publisher(topic_name, Timeseries, queue_size=1000)

        r = rospy.Rate(self.rate)
        timeseries_size = self.topic_filtering_config.timeseries_size
        sample = [0]*timeseries_size
        filter_instances = []
        for filter_count, (topic_name, msg_type, filter_class) in enumerate(self.topic_filtering_config.iter_filters()):
            filter_instances.append(filter_class())
        while not rospy.is_shutdown():

            idx = 0
            for filter_idx, filter_ins in enumerate(filter_instances):
                msg_idx = self.filter_idx_to_msg_idx[filter_idx]
                raw_msg = self.raw_msgs[msg_idx]
                if raw_msg is None:
                    rospy.logwarn("Won't publish timeseries now, since no msg is received from %s"%topic_name)
                    break
                filtered_msg = filter_ins.convert(raw_msg)
                for i in filtered_msg:
                    sample[idx] = i
                    idx += 1
            

            if idx == timeseries_size:
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                msg = Timeseries(h, sample) 
                pub.publish(msg)

            try:
                r.sleep()
            except rospy.ROSInterruptException:
                break

class OfflineRostopicsToTimeseries(RostopicsToTimeseries):
    def __init__(self, topic_filtering_config, rate):
        super(OfflineRostopicsToTimeseries, self).__init__(topic_filtering_config, rate)
        pass

    def _get_topic_msgs(self, bag, topic_name):
        if not hasattr(bag, "cache"):
            bag.cache = {}
        if topic_name not in bag.cache:
            times = []
            msgs = []            
            for topic, msg, record_t, in bag.read_messages(topics=[topic_name], start_time=None, end_time=None):
                try: 
                    t = msg.header.stamp
                except AttributeError:
                    t = record_t
                times.append(t.to_sec())
                msgs.append(msg)
            bag.cache[topic_name] = (times, msgs)

        return bag.cache[topic_name]

    def get_timeseries_mat(self, path_to_rosbag, start_time=None, end_time=None):
        if type(path_to_rosbag) == rosbag.Bag:
            bag = path_to_rosbag
        else:
            bag = rosbag.Bag(path_to_rosbag)        

        if start_time is None:
            rstart = bag.get_start_time()
        else:
            rstart = start_time.to_sec()
        if end_time is None:
            rend = bag.get_end_time()
        else:
            rend = end_time.to_sec()

        new_x = np.arange(rstart, rend, 1.0/self.rate)

        mats = []
        for topic_name, msg_type, filter_class in self.topic_filtering_config.iter_filters():
            x = []
            mat = []
            filter_ins = filter_class()

            times, msgs = self._get_topic_msgs(bag, topic_name)

            sindex = bisect.bisect(times, rstart)
            eindex = bisect.bisect(times, rend)

            x = times[sindex:eindex]
            mat = [filter_ins.convert(msgs[i]) for i in range(sindex, eindex)]
            if len(mat) == 0:
                raise Exception("No msg of topic %s in %s"%(topic_name, bag.filename))
            mat = np.array(mat)

            f = interp1d(x, mat, axis=0, kind='nearest', fill_value='extrapolate', assume_sorted=True)

            new_mat = f(new_x)

            mats.append(new_mat)

        big_mat = np.concatenate(mats, axis=1)
        return new_x, big_mat
