#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import math

class ImuOdometry:
    def __init__(self):
        rospy.init_node('imu_odometry_node')

        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.last_imu_time = rospy.Time.now()

        # Odometry pose and twist
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0 # Yaw angle in radians

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Linear velocity from acceleration (very prone to drift!)
        self.linear_vx = 0.0
        self.linear_vy = 0.0

        rospy.loginfo("IMU Odometry Node Initialized.")

    def imu_callback(self, data):
        current_time = data.header.stamp
        dt = (current_time - self.last_imu_time).to_sec()

        if dt == 0: # Avoid division by zero on first callback or if timestamps are identical
            return

        # --- 1. Process Angular Velocity for Orientation (Yaw) ---
        # Integrate Z-axis angular velocity to get yaw
        # Assuming MPU6050's Z-axis is robot's yaw axis.
        # Convert IMU's angular_velocity (rad/s) to change in yaw (delta_th)
        delta_th = data.angular_velocity.z * dt
        self.th += delta_th

        # Normalize yaw to -pi to pi range
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # --- 2. Process Linear Acceleration for Position (X, Y) ---
        # WARNING: Integrating raw acceleration for position leads to MASSIVE drift.
        # This is for demonstration, not robust navigation.

        # Get acceleration (m/s^2) in IMU's frame
        # Filter out gravity if IMU is aligned with ground plane (assuming G is mostly on Z)
        # This simplification assumes robot is flat. For full 6D, you'd rotate accels by current orientation.
        accel_x = data.linear_acceleration.x
        accel_y = data.linear_acceleration.y

        # Update linear velocities (integrate acceleration)
        self.linear_vx += accel_x * dt
        self.linear_vy += accel_y * dt

        # Update position (integrate velocity)
        # For simplicity, using linear velocities for movement in X/Y without rotating into global frame
        # A more accurate model would transform these velocities based on 'th'
        self.x += self.linear_vx * dt
        self.y += self.linear_vy * dt

        # Reset linear velocities if robot is "stopped" or for specific gait phases (complex)
        # Without encoders or more sophisticated filtering, this will drift badly.
        # A simple drift mitigation for demonstration: if accel is very low, assume no motion
        if abs(accel_x) < 0.1 and abs(accel_y) < 0.1: # Threshold for acceleration noise
            self.linear_vx = 0.0
            self.linear_vy = 0.0

        # --- 3. Publish Odometry ---
        # Create a quaternion from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # Publish the transform over TF
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),        # x, y, z position of base_link relative to odom
            odom_quat,                   # orientation of base_link relative to odom
            current_time,
            "base_link",                 # child frame_id
            "odom"                       # parent frame_id
        )

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the child frame_id for the odometry message
        odom.child_frame_id = "base_link"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # Set the twist (linear and angular velocities)
        odom.twist.twist = Twist(Vector3(self.linear_vx, self.linear_vy, 0.), Vector3(0., 0., data.angular_velocity.z))

        self.odom_pub.publish(odom)

        self.last_imu_time = current_time

if __name__ == '__main__':
    try:
        imu_odom = ImuOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
