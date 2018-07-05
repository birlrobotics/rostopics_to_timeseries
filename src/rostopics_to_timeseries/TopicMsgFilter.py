class TopicMsgFilter(object):
    def __init__(self):
        pass

    def convert(self, msg):
        raise Exception("Unimplemented")

    @staticmethod
    def vector_size():
        raise Exception("Unimplemented")

    @staticmethod
    def vector_meaning():
        raise Exception("Unimplemented")

    @staticmethod
    def get_time(msg):
        return msg.header.stamp

class BaxterEndpointStateFilter(TopicMsgFilter):
    def __init__(self):
        super(BaxterEndpointStateFilter, self).__init__()

    def convert(self, msg):
        return [\
            msg.pose.position.x,\
            msg.pose.position.y,\
            msg.pose.position.z,\
        ]

    @staticmethod
    def vector_size():
        return 3

    @staticmethod
    def vector_meaning():
        return ['pose.position.%s'%i for i in ['x', 'y', 'z']] 

class BaxterEndpointStateFilterForTwistLinear(TopicMsgFilter):
    def __init__(self):
        super(BaxterEndpointStateFilterForTwistLinear, self).__init__()

    def convert(self, msg):
        return [\
            msg.twist.linear.x,\
            msg.twist.linear.y,\
            msg.twist.linear.z,\
        ]

    @staticmethod
    def vector_size():
        return 3

    @staticmethod
    def vector_meaning():
        return ['twist.linear.%s'%i for i in ['x', 'y', 'z']] 

class WrenchStampedFilter(TopicMsgFilter):
    def __init__(self):
        super(WrenchStampedFilter, self).__init__()

    def convert(self, msg):
        return [\
            msg.wrench.force.x,\
            msg.wrench.force.y,\
            msg.wrench.force.z,\
        ]

    @staticmethod
    def vector_size():
        return 3

    @staticmethod
    def vector_meaning():
        return ['wrench.force.%s'%i for i in ['x', 'y', 'z']] 



