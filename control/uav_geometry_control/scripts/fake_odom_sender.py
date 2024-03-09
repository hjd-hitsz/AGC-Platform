import rospy
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, GetLinkStateResponse
from nav_msgs.msg import Odometry
import numpy as np

def convert_to_odom(res: GetLinkStateResponse, ref_frame, odom_frame, last_odom, hz):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = ref_frame
    odom.child_frame_id = odom_frame
    odom.pose.pose = res.link_state.pose
    p0 = np.array([last_odom.pose.pose.position.x, last_odom.pose.pose.position.y, last_odom.pose.pose.position.z])
    p1 = np.array([res.link_state.pose.position.x, res.link_state.pose.position.y, res.link_state.pose.position.z])
    v = (p1 - p0) * hz
    odom.twist.twist.linear.x = v[0]
    odom.twist.twist.linear.y = v[1]
    odom.twist.twist.linear.z = v[2]
    return odom

if __name__ == "__main__":
    rospy.init_node("fake_odom_sender")
    link_name = rospy.get_param('~link_name', 'hummingbird::base_link')
    ref_frame = rospy.get_param('~ref_frame', 'map')
    odom_frame = rospy.get_param('~odom_frame', 'base_link')
    pub_name = rospy.get_param('~odom_topic', '/odom')
    client = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    publisher = rospy.Publisher(pub_name, Odometry, queue_size=10)
    rate = rospy.Rate(50)
    last_odom = Odometry()
    while not rospy.is_shutdown():
        try:
            res = client.call(GetLinkStateRequest(link_name, ref_frame))
            odom = convert_to_odom(res, ref_frame, odom_frame, last_odom, 50)
            last_odom = odom
            publisher.publish(odom)
        except rospy.service.ServiceException as e:
            print(e)
        rate.sleep()