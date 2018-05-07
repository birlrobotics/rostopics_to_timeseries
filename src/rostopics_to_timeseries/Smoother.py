import numpy as np
from collections import deque
import copy

class Smoother(object):
    def __init__(self):
        pass

    def add_one_sample_and_get_smoothed_result(self, sample):
        raise Exception("Unimplemented")


class BoxcarSmoother(Smoother):
    window = None
    def __init__(self):
        if self.window is None:
            raise Exception("BoxcarSmoother is not meant to be instantiated. Please use function \"sized_BoxcarSmoother_factory\" to get an usable class.")

        self.size = len(self.window)
        self.convex_window = (self.window/self.window.sum()).reshape(1, -1)
        self.pool = deque()

    def add_one_sample_and_get_smoothed_result(self, sample):
        self.pool.append(copy.deepcopy(sample))
        if len(self.pool) == self.size:
            mat = np.array(self.pool)
            ret = np.matmul(self.convex_window, mat).ravel()
            self.pool.popleft()
        else:
            ret = None
        return ret
            
            
def sized_BoxcarSmoother_factory(window):
    newclass = type("BoxcarSmootherWithWindow%s"%(window), (BoxcarSmoother,),{'window': window})
    return newclass

if __name__ == '__main__':
    from scipy import signal

    try:
        b = BoxcarSmoother()
    except Exception as e:
        print e

    window = signal.triang(5)
    C = sized_BoxcarSmoother_factory(window)
    c = C()
    print str(C)
    for i in range(10):
        print i, c.add_one_sample_and_get_smoothed_result([i*i+j for j in range(7)]) 

    window = signal.boxcar(5)
    C = sized_BoxcarSmoother_factory(window)
    c = C()
    print str(C)
    for i in range(10):
        print i, c.add_one_sample_and_get_smoothed_result([i*i+j for j in range(7)]) 
