import rospy
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, GetLinkStateResponse
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import numpy.linalg as la

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

def add_odom_to_path(path: Path, odom: Odometry):
    path.header = odom.header
    if len(path.poses) > 0:
        pose_last = np.array([path.poses[-1].pose.position.x, path.poses[-1].pose.position.y, path.poses[-1].pose.position.z])
        pose_new = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
        if la.norm(pose_last - pose_new) < 0.1:
            return path
    pose = PoseStamped()
    pose.header = odom.header
    pose.pose = odom.pose.pose
    path.poses.append(pose)
    if len(path.poses) > 500:
        path.poses.pop(0)
    return path

if __name__ == "__main__":
    rospy.init_node("fake_odom_sender")
    link_name = rospy.get_param('~link_name', 'hummingbird::base_link')
    ref_frame = rospy.get_param('~ref_frame', 'map')
    odom_frame = rospy.get_param('~odom_frame', 'base_link')
    odom_topic = rospy.get_param('~odom_topic', '/odom')
    path_topic = rospy.get_param('~path_topic', '/odom_path')
    client = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
    path_pub = rospy.Publisher(path_topic, Path, queue_size=10)
    rate = rospy.Rate(50)
    last_odom = Odometry()
    path = Path()
    while not rospy.is_shutdown():
        try:
            res = client.call(GetLinkStateRequest(link_name, ref_frame))
            odom = convert_to_odom(res, ref_frame, odom_frame, last_odom, 50)
            path = add_odom_to_path(path, odom)
            last_odom = odom
            odom_pub.publish(odom)
            path_pub.publish(path)
        except rospy.service.ServiceException as e:
            print(e)
        rate.sleep()