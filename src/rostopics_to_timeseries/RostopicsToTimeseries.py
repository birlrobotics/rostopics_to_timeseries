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
from scipy import signal
from collections import deque
import logging
import ipdb

class RostopicsToTimeseries(object):
    """
    Args:
        topic_filtering_config: instance of RosTopicFilteringScheme
            containing configuration of timeseries.
    """

    def __init__(self, topic_filtering_config):
        self.msg_filters = []
        self.topic_filtering_config = topic_filtering_config
        self.rate = topic_filtering_config.rate

        self.logger = logging.getLogger("RostopicsToTimeseries")
        
        self.setup_smoothing()

    def setup_smoothing(self):
        if self.topic_filtering_config.smoother_class is not None:
            self.do_smoothing = True
            self.smoother = self.topic_filtering_config.smoother_class()
        else:
            self.do_smoothing = False

    def smooth_output(self, sample):
        if not self.do_smoothing:
            return sample
        else:
            return self.smoother.add_one_sample_and_get_smoothed_result(sample)
            

class OnlineRostopicsToTimeseries(RostopicsToTimeseries):
    def __init__(self, topic_filtering_config):
        super(OnlineRostopicsToTimeseries, self).__init__(topic_filtering_config)

        self.smooth_window_size = self.rate/2
        if self.smooth_window_size%2==0:
            self.smooth_window_size += 1
        self.centre = self.smooth_window_size/2
        self.smooth_cache = deque()
        self.timeseries_size = self.topic_filtering_config.timeseries_size
        self.running_sum = np.array([0.0]*self.timeseries_size)
        self.msg_expiration_time = 2.0/self.rate

    def _setup_listener(self):
        self.msg_buffers = []

        self.filter_idx_to_msg_idx = []
        self.filter_idx_to_topic_name = []
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
                    self.logger.warn("Message of topic %s contains no header, gonna use rospy.Time.now()"%topic_name)
                    timestamp_extractor = lambda x: rospy.Time.now()

                self.subs.append(rospy.Subscriber(
                    topic_name,
                    msg_type,
                    callback=cb,
                    callback_args=(msg_buffer, timestamp_extractor),
                ))

            self.filter_idx_to_msg_idx.append(topic_to_msg_idx[topic_name])
            self.filter_idx_to_topic_name.append(topic_name)
        
    def start_publishing_timeseries(self, topic_name):
        self._setup_listener()

        self.pub = rospy.Publisher(topic_name, Timeseries, queue_size=1000)
        rospy.loginfo("Timeseries publisher created")

        r = rospy.Rate(self.rate)
        timeseries_size = self.topic_filtering_config.timeseries_size
        sample = [0]*timeseries_size
        filter_instances = []
        for filter_count, (topic_name, msg_type, filter_class) in enumerate(self.topic_filtering_config.iter_filters()):
            filter_instances.append(filter_class())

        step_time = 1.0/self.rate

        last_ptime = None

        while not rospy.is_shutdown():
            cur_time = rospy.Time.now().to_sec()
            if cur_time == 0:
                for msg_buffer in self.msg_buffers:
                    msg_buffer.clear_all()
                last_ptime = None
                rospy.loginfo("Seems rosbag has stopped playing")
                continue

            if last_ptime is None:
                last_ptime = cur_time

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
                        rospy.logwarn("Won't publish timeseries now, since no msg of %s is received before %s "%(self.filter_idx_to_topic_name[filter_idx], ptime))
                        last_ptime = cur_time 
                        break

                    result_time, result_msg = ret
                    if result_time <= last_ptime-self.msg_expiration_time:
                        #rospy.logwarn("Won't publish timeseries now, since no msg of %s is received after last ptime %s, current ptime %s, current result_time %s"%(topic_name, last_ptime, ptime, result_time))
                        break
                    filtered_msg = filter_ins.convert(result_msg)
                    for i in filtered_msg:
                        sample[idx] = i
                        idx += 1
                

                if idx == timeseries_size:
                    h = std_msgs.msg.Header()
                    h.stamp = rospy.Time(ptime)
                    last_ptime = ptime

                    smoothed_sample = self.smooth_output(sample)
                    if smoothed_sample is not None:
                        msg = Timeseries(h, smoothed_sample) 
                        self.pub.publish(msg)
                else:
                    break

            try:
                r.sleep()
            except rospy.ROSInterruptException:
                break

class OfflineRostopicsToTimeseries(RostopicsToTimeseries):
    def __init__(self, topic_filtering_config):
        super(OfflineRostopicsToTimeseries, self).__init__(topic_filtering_config)
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
                    self.logger.warn("Message of topic %s contains no header, gonna use its recording time"%topic_name)

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
        elif type(start_time) == rospy.Time:
            rstart = start_time.to_sec()
        else:
            assert type(start_time) == float
            rstart = start_time
        if end_time is None:
            rend = bag.get_end_time()
        elif type(end_time) == rospy.Time:
            rend = end_time.to_sec()
        else:
            assert type(end_time) == float
            rend = end_time

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
                raise Exception("No msg of topic %s between %s and %s in %s"%(topic_name, rstart, rend, bag.filename))
            mat = np.array(mat)

            f = interp1d(x, mat, axis=0, kind='nearest', fill_value='extrapolate', assume_sorted=True)

            new_mat = f(new_x)


            mats.append(new_mat)

        big_mat = np.concatenate(mats, axis=1)

        self.setup_smoothing()

        smoothed_x = []
        smoothed_mat = []
        for i in range(big_mat.shape[0]):
            smoothed_sample = self.smooth_output(big_mat[i])
            if smoothed_sample is not None:
                smoothed_mat.append(smoothed_sample)
                smoothed_x.append(new_x[i])

        return smoothed_x, np.array(smoothed_mat) 
