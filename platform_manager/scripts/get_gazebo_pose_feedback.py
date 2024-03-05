import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest, GetLinkStateResponse

def convert_to_pose_stamped(res: GetLinkStateResponse):
    p = PoseStamped()
    p.header.frame_id = 'world'
    p.header.stamp = rospy.Time.now()
    p.pose = res.link_state.pose
    return p

if __name__ == '__main__':
    rospy.init_node('pose_data_feedback_node')
    link_name = rospy.get_param('~link_name', 'hummingbird::base_link')
    ref_frame = rospy.get_param('~ref_frame', 'world')
    pub_name = rospy.get_param('~pub_name', '/mavros/vision_pose/pose')
    client = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    pub_mavros_pose = rospy.Publisher(pub_name, PoseStamped, queue_size=1)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            res = client.call(GetLinkStateRequest(link_name, ref_frame))
            pose = convert_to_pose_stamped(res)
            pub_mavros_pose.publish(pose)
        except rospy.ServiceException as e:
            rospy.logerr(f"[pose_data_feedback_node] call service failed: {e}")
        rate.sleep()
        