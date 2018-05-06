import logging, coloredlogs
import threading
import bisect
import ipdb

class CircularQueue(object):
    def __init__(self, size, init_value):
        self.data_list = [init_value]*(size+1)
        self.mod = size+1
        self.head = 0
        self.tail = 0
        self.logger = logging.getLogger("CircularQueue")

    def append(self, data):
        self.data_list[self.tail] = data
        self.tail = (self.tail+1)%self.mod
        if self.tail == self.head:
            self.logger.warn("The tail just overlapped the head, will delete the head now.")
            self.head = (self.head+1)%self.mod

    def size(self):
        return (self.tail-self.head+self.mod)%self.mod

    def _check_nominal_idx(self, nominal_idx):
        if nominal_idx >= self.size():
            raise Exception("nominal_idx %s >= self.size() %s, invalid indexing."%(nominal_idx, self.size()))
        elif nominal_idx < 0:
            raise Exception("nominal_idx %s < 0, invalid indexing."%nominal_idx)

    def get(self, nominal_idx):
        self._check_nominal_idx(nominal_idx)
        return self.data_list[(self.head+nominal_idx)%self.mod]

    def clear_until(self, nominal_idx):
        self._check_nominal_idx(nominal_idx)
        self.head = (self.head+nominal_idx)%self.mod

    def clear_all(self):
        self.head = self.tail

    def if_empty(self):
        return self.head == self.tail

    def arg_bi_search_equal_or_just_smaller_than(self, data):
        if self.if_empty():
            return None

        h = self.head
        t = self.tail-1
        if t < h:
            t += self.mod

        while True:
            if t == h:
                if self.data_list[t%self.mod] <= data:
                    return (t-self.head+self.mod)%self.mod
                else:
                    return None

            c = (h+t+1)/2
            test_data = self.data_list[c%self.mod]
            if test_data == data:
                return (c-self.head+self.mod)%self.mod
            elif test_data < data:
                h = c
            else:
                t = c-1

    def log_debug_info(self):
        debug_str = "["
        for idx, data in enumerate(self.data_list):
            debug_str += str(data)
            if idx == self.head:
                debug_str += "(head)"
            if idx == self.tail:
                debug_str += "(tail)"
            debug_str += ', '
        debug_str += ']'
            
        self.logger.debug("current queue: %s. size=%s"%(debug_str, self.size()))

class MsgBuffer(object):
    def __init__(self, size):
        self.time_buffer = CircularQueue(size, 0.0)
        self.msg_buffer = CircularQueue(size, None)
        self.lock = threading.Lock()

        self.logger = logging.getLogger("MsgBuffer")

    def add_new_msg(self, time, msg):
        ''' Assume time is added in ascending order. '''

        self.lock.acquire(True)
        self.time_buffer.append(time)
        self.msg_buffer.append(msg)
        self.lock.release()


    def clear_all(self):
        self.lock.acquire(True)
        self.time_buffer.clear_all()
        self.msg_buffer.clear_all()
        self.lock.release()

    def get_msg_arriving_at_or_before_then_clear_its_precursors(self, time):
        self.lock.acquire(True)
        idx = self.time_buffer.arg_bi_search_equal_or_just_smaller_than(time)
        if idx is None:
            self.logger.warn("Cannot find a message whose arrival time is equal or just smaller than %s"%time)
            ret = None
        else:
            result_time = self.time_buffer.get(idx)
            result_msg = self.msg_buffer.get(idx)
            self.time_buffer.clear_until(idx)
            self.msg_buffer.clear_until(idx)
            ret = (result_time, result_msg)
        self.lock.release()
        return ret

    def log_debug_info(self):
        self.time_buffer.log_debug_info()
        self.msg_buffer.log_debug_info()

def test_CircularQueue():
    c = CircularQueue(5, None)
    c.log_debug_info()
    for i in range(10):
        c.append(i)
        c.log_debug_info()

    for i in np.arange(4, 11, 0.5):
        idx = c.arg_bi_search_equal_or_just_smaller_than(i) 
        if idx is None:
            data = None
        else:
            data = c.get(idx)
        logger.info("The item that is equal or just smaller than %s is %s"%(i, data))
        logger.info("nominal_idx=%s"%idx)
    try:
        c.get(5)
    except Exception as e:
        logger.error(e)

    try:
        c.get(-1)
    except Exception as e:
        logger.error(e)

def test_MsgBuffer():
    import time
    mb = MsgBuffer(5)
    def func_1():
        mb.log_debug_info()
        for i in range(10):
            mb.add_new_msg(i, "data received at %s sec"%i)
            mb.log_debug_info()
            time.sleep(2)

    t1 = threading.Thread(target=func_1)
    t1.daemon = True
    t1.start()
        
    
    def func_2():
        mb.log_debug_info()
        for i in np.arange(4, 11, 0.5):
            ret = mb.get_msg_arriving_at_or_before_then_clear_its_precursors(i)
            logger.info("msg_arriving_at_or_before %s: %s"%(i , ret))
            time.sleep(1)

    t2 = threading.Thread(target=func_2)
    t2.daemon = True
    t2.start()

    t1.join()
    t2.join()

if __name__ == '__main__':
    import numpy as np

    coloredlogs.install()
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.DEBUG)
    logger.addHandler(consoleHandler)
    
    #test_CircularQueue()
    test_MsgBuffer()
