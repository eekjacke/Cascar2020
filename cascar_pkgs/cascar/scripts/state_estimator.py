#!/usr/bin/env python3

import sympy as sym
import rospy
import tf
import tf2_ros as tf2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import TransformStamped
from cascar.msg import CarSensor

class StateEstimator:
    """ Simple implementation of an Extended Kalman Filter. 
        The following class supplies skeleton code for experimenting
        with state estimation for the course TSFS12."""
    
    X = sym.Matrix([[0.], [0.], [0.]]) # x, y, th, biasw
    
    dt = 0.1 # Sample time
    bias_w = -0.0232720 # Bias in gyro measurements, this could vary from car to car..
    offset_comp = 1 #1.35 
    
    L = 0.275 # Wheel base
    
    # Example values for filter properties.
    P = 0.01*sym.eye(3) # Initial uncertainty
    Q = 0.1*sym.eye(3) # Process noise matrix

    H = None
    F = None
    
    fs = 1/dt # Sample rate
    
    # Inputs to process model
    w_in = 0
    v_in = 0
    
    chi_sqr_tab = [3.84, 5.99, 7.81] # Used for outlier detection
    
    pose_initialized = True
    
    def __init__(self):
        if rospy.has_param('qualisys_used'):
            self.pose_initialized = False
        
        rospy.init_node('state_estimator', anonymous=True)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=100)
        self.odom_frame_broadcaster = tf2.TransformBroadcaster()
        static_frame_broadcaster = tf2.StaticTransformBroadcaster()
        
        laser_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        laser_frame = TransformStamped()
        laser_frame.header.stamp = rospy.Time.now()
        laser_frame.header.frame_id = 'base_link'
        laser_frame.child_frame_id = 'laser_link'

        laser_frame.transform.translation.x = 0.0
        laser_frame.transform.translation.y = 0.0
        laser_frame.transform.translation.z = 0.0

        laser_frame.transform.rotation.x = laser_quat[0]
        laser_frame.transform.rotation.y = laser_quat[1]
        laser_frame.transform.rotation.z = laser_quat[2]
        laser_frame.transform.rotation.w = laser_quat[3]
        
        
        map_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        map_frame = TransformStamped()
        map_frame.header.stamp = rospy.Time.now()
        map_frame.header.frame_id = 'map'
        map_frame.child_frame_id = 'odom'

        map_frame.transform.translation.x = 0.0
        map_frame.transform.translation.y = 0.0
        map_frame.transform.translation.z = 0.0

        map_frame.transform.rotation.x = map_quat[0]
        map_frame.transform.rotation.y = map_quat[1]
        map_frame.transform.rotation.z = map_quat[2]
        map_frame.transform.rotation.w = map_quat[3]
        
        static_frame_broadcaster.sendTransform([laser_frame, map_frame])
        
        self.gen_jacobians()
        self.run()
        
    def reinit_pose(self, Xnew):
        """ Reset the initial pose."""
        self.X = sym.Matrix([[Xnew[0]], [Xnew[1]], [Xnew[2]]])
    
    def gen_jacobians(self):
        """ Generate symbolic expression for all the Jacobians in the filter. """
        dt, v, th, L, w, df = sym.symbols('d_t, v, \Theta, L, \omega, \delta_f')
        
        self.H = sym.Matrix([[1, 0, 0],
                             [0, 1, 0],
                             [0, 0, 1]])
        
        self.F = sym.Matrix([[1, 0, -dt*v*sym.sin(th)],
                             [0, 1, dt*v*sym.cos(th)], 
                             [0, 0, 1]])
    
    def runge_kutta(self, func, X, U, dt):
        """ RK4 """
        k1 = func(X, U)
        k2 = func(X + dt / 2 * k1, U)
        k3 = func(X + dt / 2 * k2, U)
        k4 = func(X + dt * k3, U)
        return X + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4) 
    
    def euler_forward(self, func, X, U, dt):
        return X + func(X,U)*dt
    
    def process_model(self, X, U):
        """ Kinematic bicycle model with velocity and yaw rate as input. """
        x = U[0] * sym.cos(X[2])
        y = U[0] * sym.sin(X[2])
        th = U[1]*self.offset_comp - self.bias_w
        return sym.Matrix([[x], [y], [th]])
    
    def time_update(self, U):
        """ Time update part of Kalman filter. """
        self.X = self.runge_kutta(self.process_model, self.X, U, self.dt)
        
        J = self.F.subs({'d_t': self.dt, 'v': U[0], '\Theta': self.X[2]})
        
        self.P = J*self.P*J.T + self.Q
        
    def measurement_update(self, H, Z, R):
        """ Calculation part of measurement update. """
        S = R + H*self.P*H.T
        iS = S**-1
        K = self.P*H.T*iS
        
        self.X = self.X + K*Z
        self.P = self.P - K*H*self.P
        
    def outlier_detection(self, yk, R, H):
        """ Outlier detection as formulated by Gustafsson (Sensor Fusion) """
        D = (yk - H*self.X)
        L = (H*self.P*H.T + R)
        iL = L**-1
        T = D.T * iL * D
        return T[0]
        
    def qualisys_callback(self, data):
        """ Callback function when measurements are available from Qualisys. """
        current_time = rospy.Time.now()
        
        pose_q = data.pose.pose.position
        xM = pose_q.x
        yM = pose_q.y
        
        """ Uncomment to get velocity data """
#         vel_q = data.twist.twist
#         vx = vel_q.linear.x
#         vy = vel_q.linear.y
#         vM = sym.sqrt(vx*vx + vy*vy)
#         wM = vel_q.angular.z
        
        ori_q = data.pose.pose.orientation
        ori_list = [ori_q.x, ori_q.y, ori_q.z, ori_q.w]
        (_, _, thM) = tf.transformations.euler_from_quaternion(ori_list)

        if not self.pose_initialized:
            self.reinit_pose([xM, yM, thM])
            self.pose_initialized = True
        else:
            """ Currently, the states are taken 'as is'. This could be experimented with
                by tuning the filter parameters. """
            
            self.X[0] = xM
            self.X[1] = yM
            self.X[2] = thM
            
#             H = self.H

#             yk = sym.Matrix([[xM],[yM],[thM]])

#             R = 0.001*sym.eye(3) # We are pretty confident about the measurements from Q

#             #T = self.outlier_detection(yk, R, H)

#             if T < self.chi_sqr_tab[2]:

#                 # Innovation
#                 Z = sym.Matrix([[xM - self.X[0]],
#                                 [yM - self.X[1]],
#                                 [thM - self.X[2]]])

#                 self.measurement_update(H,Z,R)
                
                

    def imu_callback(self, data):
        """ Callback function when measurements are available from IMU"""
        atM = data.a_x
        anM = data.a_y
        wM = data.w
        self.w_in = wM
    
    def cascar_callback(self, data):
        """ Callback function when measurements are available from the vehicle"""
        dfM = data.df
        vM = data.v
        self.v_in = vM
            
    def broadcast_odometry_message(self, pose, current_time):
        """ Publish the current pose on the /odom topic """
        x, y, th, vx, vy, vth = pose

        # first, the transform over tf
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = current_time
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'

        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = 0.0

        transform_stamped.transform.rotation.x = odom_quat[0]
        transform_stamped.transform.rotation.y = odom_quat[1]
        transform_stamped.transform.rotation.z = odom_quat[2]
        transform_stamped.transform.rotation.w = odom_quat[3]

        self.odom_frame_broadcaster.sendTransform(transform_stamped)

        # next, publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the pose
        odom.pose.pose = Pose(Point(x, y, 0.),
                              Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0),
                                 Vector3(0, 0, vth))

        self.odom_pub.publish(odom)
        
    def broadcast_odom_frame(self, pose, current_time):
        """ Alternative pose publisher that assumes stand-still. """
        x, y, th, vx, vy, vth = pose
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = current_time
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'

        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = 0.0

        transform_stamped.transform.rotation.x = odom_quat[0]
        transform_stamped.transform.rotation.y = odom_quat[1]
        transform_stamped.transform.rotation.z = odom_quat[2]
        transform_stamped.transform.rotation.w = odom_quat[3]

        self.odom_frame_broadcaster.sendTransform(transform_stamped)
        
    def run(self):
        """ Main function. Sets up subscribers and executes the head functionality. """
        rospy.Subscriber('qualisys/cascar/odom', Odometry, self.qualisys_callback, queue_size=1)
        rospy.Subscriber('sensor/imu', CarSensor, self.imu_callback, queue_size=1)
        rospy.Subscriber('sensor/cascar', CarSensor, self.cascar_callback, queue_size=1)
        rate = rospy.Rate(self.fs)
        while not self.pose_initialized:
            # If we're using qualisys, wait for the first measurement to use as initial pose.
            rate.sleep()
            
        while not rospy.is_shutdown():
            # Collect pose data and publish continuously.
            pose = (self.X[0],
                    self.X[1],
                    self.X[2],
                    self.v_in*sym.cos(self.X[2]),
                    self.v_in*sym.sin(self.X[2]),
                    self.w_in)

            current_time = rospy.Time.now()
            self.broadcast_odometry_message(pose, current_time)
            
            self.time_update([self.v_in, self.w_in])
            self.v_in = 0

            rate.sleep()
            
            
def main():
    EKF = StateEstimator()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
