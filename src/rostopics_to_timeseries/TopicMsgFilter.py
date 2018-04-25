class TopicMsgFilter(object):
    def __init__(self):
        pass

    def convert(self, msg):
        raise Exception("Unimplemented")

    def vector_size(self):
        raise Exception("Unimplemented")

    def vector_meaning(self):
        raise Exception("Unimplemented")

    def get_time(self, msg):
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

    def vector_size(self):
        return 3

    def vector_meaning(self):
        return ['pose.position.%s'%i for i in ['x', 'y', 'z']] 

class WrenchStampedFilter(TopicMsgFilter):
    def __init__(self):
        super(WrenchStampedFilter, self).__init__()

    def convert(self, msg):
        return [\
            msg.wrench.force.x,\
            msg.wrench.force.y,\
            msg.wrench.force.z,\
        ]

    def vector_size(self):
        return 3

    def vector_meaning(self):
        return ['wrench.force.%s'%i for i in ['x', 'y', 'z']] 



