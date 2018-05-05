import rospy
import threading
from rostopics_to_timeseries.msg import Timeseries
import copy
import numpy as np
import std_msgs.msg
import rosbag
from scipy.interpolate import interp1d
import bisect
from FastMsgBuffer import MsgBuffer
import ipdb

class RostopicsToTimeseries(object):
    """
    Args:
        topic_filtering_config: [TODO]
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
        self.msg_buffers = []

        self.filter_idx_to_msg_idx = []
        self.subs = []
        def cb(msg, callback_args):
            msg_buffer, timestamp_extractor = callback_args
            arrival_time = timestamp_extractor(msg).to_sec()
            msg_buffer.add_new_msg(arrival_time, msg)

        topic_to_msg_idx = {}
        for filter_count, (topic_name, msg_type, filter_class) in enumerate(self.topic_filtering_config.iter_filters()):
            if topic_name not in topic_to_msg_idx:
                msg_buffer = MsgBuffer(10000)
                self.msg_buffers.append(msg_buffer)
                topic_to_msg_idx[topic_name] = len(self.msg_buffers)-1

                try:
                    filter_class.get_time(msg_type())
                    timestamp_extractor = filter_class.get_time
                except:
                    timestamp_extractor = lambda x: rospy.Time.now()

                self.subs.append(rospy.Subscriber(
                    topic_name,
                    msg_type,
                    callback=cb,
                    callback_args=(msg_buffer, timestamp_extractor),
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

        step_time = 1.0/self.rate

        last_ptime = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            cur_time = rospy.Time.now().to_sec()

            ptime = last_ptime+step_time
            ptimes = []
            while ptime <= cur_time:
                ptimes.append(ptime)
                ptime += step_time
                
            for ptime in ptimes:
                idx = 0
                for filter_idx, filter_ins in enumerate(filter_instances):
                    msg_idx = self.filter_idx_to_msg_idx[filter_idx]
                    msg_buffer = self.msg_buffers[msg_idx]

                    ret = msg_buffer.get_msg_arriving_at_or_before_then_clear_its_precursors(ptime)

                    if ret is None:
                        rospy.logwarn("Won't publish timeseries now, since no msg of %s is received before %s "%(topic_name, ptime))
                        break

                    result_time, result_msg = ret
                    filtered_msg = filter_ins.convert(result_msg)
                    for i in filtered_msg:
                        sample[idx] = i
                        idx += 1
                

                if idx == timeseries_size:
                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time(ptime)
                    last_ptime = ptime
                    msg = Timeseries(h, sample) 
                    pub.publish(msg)
                else:
                    last_ptime = cur_time 
                    break


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
