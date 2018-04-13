class TopicMsgToVector(object):
    def __init__(self):
        pass

    def convert(self, msg):
        raise Exception("Unimplemented")

    def vector_size(self):
        raise Exception("Unimplemented")

    def vector_meaning(self):
        raise Exception("Unimplemented")

class BaxterEndpointStateToVector(TopicMsgToVector):
    def __init__(self):
        super(BaxterEndpointStateToVector, self).__init__()
        self.recyclable_list = [None]*self.vector_size()

    def convert(self, msg):
        self.recyclable_list[0] = msg.pose.position.x
        self.recyclable_list[1] = msg.pose.position.y
        self.recyclable_list[2] = msg.pose.position.z
        return self.recyclable_list

    def vector_size(self):
        return 3

    def vector_meaning(self):
        return ['pose.position.%s'%i for i in ['x', 'y', 'z']] 

class WrenchStampedToVector(TopicMsgToVector):
    def __init__(self):
        super(WrenchStampedToVector, self).__init__()
        self.recyclable_list = [None]*self.vector_size()

    def convert(self, msg):
        self.recyclable_list[0] = msg.wrench.force.x
        self.recyclable_list[1] = msg.wrench.force.y
        self.recyclable_list[2] = msg.wrench.force.z
        return self.recyclable_list

    def vector_size(self):
        return 3

    def vector_meaning(self):
        return ['wrench.force.%s'%i for i in ['x', 'y', 'z']] 
