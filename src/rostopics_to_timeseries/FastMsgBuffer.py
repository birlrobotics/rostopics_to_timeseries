import logging, coloredlogs
import threading
import bisect
import ipdb

class CircularQueue(object):
    def __init__(self, size):
        self.data_list = [None]*(size+1)
        self.mod = size+1
        self.head = 0
        self.tail = 0
        self.logger = logging.getLogger("FastMsgBuffer")

    def append(self, data):
        self.data_list[self.tail] = data
        self.tail = (self.tail+1)%self.mod
        if self.tail == self.head:
            self.logger.warn("The tail just overlapped the head, will popped the head now.")
            self.head = (self.head+1)%self.mod

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
                    return t%self.mod
                else:
                    return None

            c = (h+t+1)/2
            test_data = self.data_list[c%self.mod]
            if test_data == data:
                return c%self.mod
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
            
        self.logger.debug("current queue: %s"%debug_str)

class MsgBuffer(object):
    def __init__(self, size):
        self.time_buffer = CircularQueue(size)
        self.msg_buffer = CircularQueue(size)
        self.lock = threading.Lock()

    def add_new_msg(self, time, msg):
        ''' Assume time is added in ascending order. '''

        self.lock.acquire(True)
        self.time_buffer.append(time)
        self.msg_buffer.append(msg)
        self.lock.release()

    def get_first_data_later_than(self, time):
        self.lock.acquire(True)
        pass
        self.lock.release()


if __name__ == '__main__':
    import numpy as np

    coloredlogs.install()
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel(logging.DEBUG)
    logger.addHandler(consoleHandler)
    
    c = CircularQueue(5) 
    for i in range(10):
        c.append(i)
    c.log_debug_info()

    for i in np.arange(3, 13, 0.5):
        idx = c.arg_bi_search_equal_or_just_smaller_than(i) 
        if idx is None:
            data = None
        else:
            data = c.data_list[idx]
        logger.info("The item that is equal or just smaller than %s is %s"%(i, data))

