import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse, CommandBool, CommandBoolRequest, CommandBoolResponse
from geometry_msgs.msg import PoseStamped

class FlightControl:
    def __init__(self) -> None:
        self.last_req_time = rospy.get_time()
        self.current_state = State()
        self.rate = rospy.Rate(50)
        self.count = 0
        
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.local_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        
        self.init_pose = PoseStamped()
        self.init_pose.pose.position.x = rospy.get_param("~init_x", 0)
        self.init_pose.pose.position.y = rospy.get_param("~init_y", 0)
        self.init_pose.pose.position.z = rospy.get_param("~init_z", 1.6)
        self.init_pose.pose.orientation.x = 0
        self.init_pose.pose.orientation.y = 0
        self.init_pose.pose.orientation.z = 0
        self.init_pose.pose.orientation.w = 1
        
        self.hover_pose = PoseStamped()
        self.hover_pose.pose.position.x = rospy.get_param("~hover_x", 0.5)
        self.hover_pose.pose.position.y = rospy.get_param("~hover_y", 0)
        self.hover_pose.pose.position.z = rospy.get_param("~hover_z", 1.17)
        self.hover_pose.pose.orientation.x = 0
        self.hover_pose.pose.orientation.y = 0
        self.hover_pose.pose.orientation.z = 0
        self.hover_pose.pose.orientation.w = 1
        
    def update_state(self, msg: State):
        self.current_state = msg
        if rospy.is_shutdown():
            return
        if not self.if_fcu_connected():
            self.rate.sleep()
            return
        if self.count < 2:
            self.local_pub.publish(self.init_pose)
            self.count += 1
            self.rate.sleep()
            self.last_req_time = rospy.get_time()
            return
        if not self.if_offboard():
            self.rate.sleep()
            return
        if not self.if_armed():
            self.rate.sleep()
            return
        self.local_pub.publish(self.hover_pose)
        self.rate.sleep()
        return
        
    def if_offboard(self):
        if self.current_state.mode == "OFFBOARD":
            return True
        else:
            if rospy.get_time() - self.last_req_time > 5:
                try:
                    res = self.set_mode_client.call(SetModeRequest(custom_mode="OFFBOARD"))
                    if res.mode_sent:
                        rospy.loginfo("[takeoff] set offboard mode")
                        self.last_req_time = rospy.get_time()
                        return False
                except rospy.ServiceException as e:
                    rospy.logerr(f"[takeoff] call [SetMode] service failed: {e}")
                    return False
    def if_armed(self):
        if self.current_state.armed:
            return True
        else:
            if rospy.get_time() - self.last_req_time > 5:
                try:
                    res = self.arm_client.call(CommandBoolRequest(value=True))
                    if res.success:
                        rospy.loginfo("[takeoff] vehicle armed")
                        self.last_req_time = rospy.get_time()
                        return False
                except rospy.ServiceException as e:
                    rospy.logerr(f"[takeoff] call [CommandBool] service failed: {e}")
                    return False
    def if_fcu_connected(self):
        if self.current_state.connected:
            return True
        else:
            return False
    
if __name__ == "__main__":
    rospy.init_node("takeoff")
    fc = FlightControl()
    state_sub = rospy.Subscriber("/mavros/state", State, fc.update_state, queue_size=1)
    rospy.spin()
    