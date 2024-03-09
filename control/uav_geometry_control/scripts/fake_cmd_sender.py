import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import math
import numpy.linalg as la
from uav_geometry_control.msg import flatness_polycoeffs
import time
from std_msgs.msg import Float64
import transforms3d.quaternions as tq
import random as ran

def get_yaw(q):
    w, x, y, z = q
    sin = 2 * (w * z + x * y)
    cos = 1 - 2 * (y * y + z * z)
    return math.atan2(sin, cos)

def get_flatness_attitude(a, yaw):
    b3 = a + np.array([0, 0, 9.81])
    b3 /= la.norm(b3)
    b1c = np.array([math.cos(yaw), math.sin(yaw), 0])
    b2 = np.cross(b3, b1c) / la.norm(np.cross(b3, b1c))
    b1 = np.cross(b2, b3)
    R = np.array([b1, b2, b3]).T
    q = tq.mat2quat(R)
    return q
    
class GenerateMinJerk:
    def __init__(self, v_max, a_max, yaw_dot_max) -> None:
        self.odom = Odometry()
        self.v_max = v_max
        self.a_max = a_max
        self.min_dist = v_max * v_max / a_max
        self.min_dist_time = 2 * v_max / a_max
        self.yaw_dot_max = yaw_dot_max
        self.start_header = self.odom.header
        self.start_header.stamp = rospy.Time.now()
        self.path = Path()
        hover_x = rospy.get_param('~takeoff/hover_x', 0.5)
        hover_y = rospy.get_param('~takeoff/hover_y', 0)
        self.hover_z = rospy.get_param('~takeoff/hover_z', 1.17)
        self.coeffs = [np.zeros(6) for i in range(4)]
        self.coeffs[0][0] = hover_x
        self.coeffs[1][0] = hover_y
        self.coeffs[2][0] = self.hover_z
        self.duration = 0
        flatness_polycoeffs_topic = rospy.get_param('~flatness_polycoeffs_topic', '/flatness_polycoeffs')
        path_topic = rospy.get_param('~path_topic', '/minJerk_path')
        self.coeffs_pub = rospy.Publisher(flatness_polycoeffs_topic, flatness_polycoeffs, queue_size=1)
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=1)
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
        v0 = np.array([self.odom.twist.twist.linear.x,
                       self.odom.twist.twist.linear.y,
                       self.odom.twist.twist.linear.z])
        pf = np.array([msg.pose.position.x, msg.pose.position.y, self.hover_z + 4 * ran.random() - 2])
        yaw0 = get_yaw(q0)
        yawf = get_yaw(qf)
        t0 = time.time()
        self.duration = self.transfer_time(yaw0, yawf, p0, pf)
        self.coeffs[0] = self.minJerk(p0[0], v0[0], pf[0], self.duration)
        self.coeffs[1] = self.minJerk(p0[1], v0[0], pf[1], self.duration)
        self.coeffs[2] = self.minJerk(p0[2], v0[0], pf[2], self.duration)
        self.coeffs[3] = self.minJerk(yaw0, 0, yawf, self.duration)
        t_use = time.time() - t0
        rospy.loginfo(f"compute minJerk trajectory, duration: {self.duration}s, time usage: {t_use}s, goal: {[pf[0], pf[1], pf[2], yawf]}")
        self.start_header = self.odom.header
        # publish planning path
        t_span = np.linspace(0, self.duration, int(self.duration / 0.05))
        self.path = Path()
        self.path.header = self.odom.header
        for t in t_span:
            pt = np.zeros(3)
            at = np.zeros(3)
            for i in range(4):
                c0, c1, c2, c3, c4, c5 = self.coeffs[i]
                if i < 3:
                    pt[i] = c0 + c1 * t + c2 * t**2 + c3 * t**3 + c4 * t**4 + c5 * t**5
                    at[i] = 2*c2 + 6*c3*t + 12*c4*t**2 + 20*c5*t**3
                else:
                    yawt = c0 + c1 * t + c2 * t**2 + c3 * t**3 + c4 * t**4 + c5 * t**5
            q = get_flatness_attitude(at, yawt)
            pose = PoseStamped()
            pose.header = self.odom.header
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = pt[2]
            pose.pose.orientation.w = q[0]
            pose.pose.orientation.x = q[1]
            pose.pose.orientation.y = q[2]
            pose.pose.orientation.z = q[3]
            self.path.poses.append(pose)
            
    def transfer_time(self, yaw0, yawf, p0, pf):
        t_yaw = abs(yaw0 - yawf) / self.yaw_dot_max * 2
        dist = la.norm(pf - p0)
        t_p = self.min_dist_time
        if dist > self.min_dist:
            t_p = self.min_dist_time + (dist - self.min_dist) / self.v_max
        else:
            t_p = math.sqrt(dist / self.a_max)
        # rospy.loginfo(f"yaw transfer time: {t_yaw}, p transfer time: {t_p}")
        return max(t_yaw, t_p)
    def minJerk(self, x0, v0, xf, t):
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
        b[1] = v0
        return la.solve(A, b)
    def send_coeffs_timer(self, event):
        coeffs = flatness_polycoeffs()
        coeffs.dim = 5
        coeffs.tf = self.duration
        coeffs.x_coeff = [Float64(c) for c in self.coeffs[0]]
        coeffs.y_coeff = [Float64(c) for c in self.coeffs[1]]
        coeffs.z_coeff = [Float64(c) for c in self.coeffs[2]]
        coeffs.yaw_coeff = [Float64(c) for c in self.coeffs[3]]
        coeffs.header = self.start_header
        self.coeffs_pub.publish(coeffs)
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('test_node')
    odom_topic = rospy.get_param('~odom_topic', '/odom')
    v_max = rospy.get_param('~minJerk/v_max', 0.5)
    a_max = rospy.get_param('~minJerk/a_max', 0.5)
    yaw_dot_max = rospy.get_param('~minJerk/yaw_dot_max', 0.5)
    generator = GenerateMinJerk(v_max, a_max, yaw_dot_max)
    odom_sub = rospy.Subscriber(odom_topic, Odometry, generator.recv_odom)
    goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, generator.recv_nav_goal)
    cmd_timer = rospy.Timer(rospy.Duration(0.02), generator.send_coeffs_timer)
    rospy.spin()