import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
import numpy.linalg as la
from uav_geometry_control.msg import flatness_polycoeffs
import time as t
from std_msgs.msg import Float64

def get_yaw(q):
    w, x, y, z = q
    sin = 2 * (w * z + x * y)
    cos = 1 - 2 * (y * y + z * z)
    return math.atan2(sin, cos)
    
class GenerateMinJerk:
    def __init__(self, v_max, yaw_dot_max) -> None:
        self.odom = Odometry()
        self.v_max = v_max
        self.yaw_dot_max = yaw_dot_max
        hover_x = rospy.get_param('~takeoff/hover_x', 0.5)
        hover_y = rospy.get_param('~takeoff/hover_y', 0)
        hover_z = rospy.get_param('~takeoff/hover_z', 1.17)
        self.coeffs = [np.zeros(6) for i in range(4)]
        self.coeffs[0][0] = hover_x
        self.coeffs[1][0] = hover_y
        self.coeffs[2][0] = hover_z
        self.duration = 0
        flatness_polycoeffs_topic = rospy.get_param('~flatness_polycoeffs_topic', '/flatness_polycoeffs')
        self.coeffs_pub = rospy.Publisher(flatness_polycoeffs_topic, flatness_polycoeffs, queue_size=1)
    def recv_odom(self, msg):
        self.odom = msg
    def recv_nav_goal(self, msg):
        qf = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        q0 = np.array([self.odom.pose.pose.orientation.w,
                       self.odom.pose.pose.orientation.x,
                       self.odom.pose.pose.orientation.y,
                       self.odom.pose.pose.orientation.z])
        p0 = np.array([self.odom.pose.pose.position.x,
                       self.odom.pose.pose.position.y,
                       self.odom.pose.pose.position.z])
        pf = np.array([msg.pose.position.x, msg.pose.position.y, self.odom.pose.pose.position.z])
        yaw0 = get_yaw(q0)
        yawf = get_yaw(qf)
        t0 = t.time()
        self.duration = self.transfer_time(yaw0, yawf, p0, pf)
        self.coeffs[0] = self.minJerk(p0[0], pf[0], self.duration)
        self.coeffs[1] = self.minJerk(p0[1], pf[1], self.duration)
        self.coeffs[2] = self.minJerk(p0[2], pf[2], self.duration)
        self.coeffs[3] = self.minJerk(yaw0, yawf, self.duration)
        t_use = t.time() - t0
        rospy.loginfo(f"compute minJerk trajectory, duration: {self.duration}s, time usage: {t_use}s, goal: {[pf[0], pf[1], pf[2], yawf]}")
        t_span = np.linspace(0, self.duration, math.ceil(self.duration / 0.1))
        x = [self.coeffs[0][0] + self.coeffs[0][1] * t + self.coeffs[0][2] * t**2 + self.coeffs[0][3] * t**3 + self.coeffs[0][4] * t**4 + self.coeffs[0][5] * t**5 for t in t_span]
        y = [self.coeffs[1][0] + self.coeffs[1][1] * t + self.coeffs[1][2] * t**2 + self.coeffs[1][3] * t**3 + self.coeffs[1][4] * t**4 + self.coeffs[1][5] * t**5 for t in t_span]
        z = [self.coeffs[2][0] + self.coeffs[2][1] * t + self.coeffs[2][2] * t**2 + self.coeffs[2][3] * t**3 + self.coeffs[2][4] * t**4 + self.coeffs[2][5] * t**5 for t in t_span]
        points = np.array([x, y, z]).T
        np.savetxt("/home/hjd-nrsl/code/AGC_Platform_ws/src/AGC-Platform/control/uav_geometry_control/data/point.txt", points)
    def transfer_time(self, yaw0, yawf, p0, pf):
        return max(abs(yaw0 - yawf) / self.yaw_dot_max * 2, la.norm(pf - p0) / self.v_max)
    def minJerk(self, x0, xf, t):
        A = np.zeros((6, 6))
        A[0, 0] = 1
        A[1, 1] = 1
        A[2, 2] = 2
        A[3, :] = np.array([1, t, t**2, t**3, t**4, t**5])
        A[4, :] = np.array([0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4])
        A[5, :] = np.array([0, 0, 2, 6*t, 12*t**2, 20*t**3]) 
        b = np.zeros(6)
        b[0] = x0
        b[3] = xf
        return la.solve(A, b)
    def send_coeffs_timer(self, event):
        coeffs = flatness_polycoeffs()
        coeffs.dim = 5
        coeffs.tf = self.duration
        coeffs.x_coeff = [Float64(c) for c in self.coeffs[0]]
        coeffs.y_coeff = [Float64(c) for c in self.coeffs[1]]
        coeffs.z_coeff = [Float64(c) for c in self.coeffs[2]]
        coeffs.yaw_coeff = [Float64(c) for c in self.coeffs[3]]
        coeffs.header = self.odom.header
        self.coeffs_pub.publish(coeffs)

if __name__ == '__main__':
    rospy.init_node('test_node')
    odom_topic = rospy.get_param('~odom_topic', '/odom')
    v_max = rospy.get_param('~minJerk/v_max', 0.5)
    yaw_dot_max = rospy.get_param('~minJerk/yaw_dot_max', 0.5)
    generator = GenerateMinJerk(v_max=0.5, yaw_dot_max=0.5)
    odom_sub = rospy.Subscriber(odom_topic, Odometry, generator.recv_odom)
    goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, generator.recv_nav_goal)
    cmd_timer = rospy.Timer(rospy.Duration(0.02), generator.send_coeffs_timer)
    rospy.spin()