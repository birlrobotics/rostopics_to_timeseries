from rostopics_to_timeseries.RostopicsToTimeseries import OfflineRostopicsToTimeseries
from rostopics_to_timeseries.TopicMsgToVector import TopicMsgToVector, BaxterEndpointStateToVector
import baxter_core_msgs.msg
import rospy
import matplotlib.pyplot as plt

if __name__ == '__main__':
    topic_info = [
        (
            "/robot/limb/right/endpoint_state", 
            baxter_core_msgs.msg.EndpointState, 
            BaxterEndpointStateToVector,
        ),
    ] 
    ofrt = OfflineRostopicsToTimeseries(topic_info, rate=10) 
    t, mat = ofrt.get_timeseries_mat("test_offline.bag")

    dimension = mat.shape[1] 
    fig, axs = plt.subplots(nrows=dimension, ncols=1)
    if dimension == 1:
        axs = [axs]

    for dim_idx in range(dimension):
        ax = axs[dim_idx]
        x = t
        y = mat[:, dim_idx]
        ax.plot(x, y, 'ro')

    plt.show()
