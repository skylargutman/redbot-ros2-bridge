#!/usr/bin/env python3
"""
RedBot Odemetry Node
Calculates robot pose from wheel encoder data
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdometryNode(Node):
	"""Calculates odometry from wheel encoders"""

	def __init__(self):
		super().__init__('odometry_node')

		# Robot physical parameters 
		self.declare_parameter('wheel_diameter', 0.065) #meters
		self.declare_parameter('wheel_base', 0.15) #meters
		self.declare_parameter('ticks_per_revolution', 192) #encoder ticks

		#get parameters
		self.wheel_diameter = self.get_parameter('wheel_diameter').value
		self.wheel_base = self.get_parameter('wheel_base').value
		self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value

		#calculate distance per tick
		wheel_circumference = math.pi * self.wheel_diameter
		self.meters_per_tick = wheel_circumference / self.ticks_per_rev

		#robot pose (position and Orientation)
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0 #heading in radians

		#previous encoder values
		self.last_left_ticks = 0
		self.last_right_ticks = 0
		self.first_reading = True

		#time tracking
		self.last_time = self.get_clock().now()

		#Subscriber - listen to encoder data
		self.encoder_sub = self.create_subscription(
			Int32MultiArray,
			'encoders',
			self.encoder_callback,
			10
		)

		#Publisher - odometry messages
		self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

		# TF broadcaster - publishes coordinate transforms
		self.tf_broadcaster = TransformBroadcaster(self)

		self.get_logger().info('Odometry node started')
		self.get_logger().info(f'wheel diameter: {self.wheel_diameter}m')
		self.get_logger().info(f'Wheel base: {self.wheel_base}m')
		self.get_logger().info(f'Meters per tick: {self.meters_per_tick:.6f}m')

	def encoder_callback(self,msg):
		"""Process encoder data and update odometry"""

		#extract encoder values
		if len(msg.data) < 2:
			return

		left_ticks = msg.data[0]
		right_ticks = msg.data[1]

		#skip first reading (no previous data to compare)
		if self.first_reading:
			self.last_left_ticks = left_ticks
			self.last_right_ticks = right_ticks
			self.first_reading = False
			return

		#calculate change in ticks since last reading
		delta_left = left_ticks - self.last_left_ticks
		delta_right = right_ticks - self.last_right_ticks

		#update last tick values
		self.last_left_ticks = left_ticks
		self.last_right_ticks = right_ticks

		#convert ticks to meters
		left_distance = delta_left * self.meters_per_tick
		right_distance = delta_right * self.meters_per_tick


		#differential drive kinematics
		#Caluculate distance traveled and change in heading
		distance = (left_distance + right_distance) / 2.0
		delta_theta = (right_distance - left_distance) / self.wheel_base

		#Update robot pose
		#If going straight, simple addition
		if abs(delta_theta) < 0.0001:
			self.x += distance * math.cos(self.theta)
			self.y += distance * math.sin(self.theta)
		else:
			#if turning, use arc calculations
			radius = distance / delta_theta
			self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
			self.y += radius * (-math.cos(self.theta + delta_theta) + math.cos(self.theta))
		self.theta += delta_theta

		#Normalize theta to [-pi, pi]
		self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))


		#calculate the time delta
		current_time = self.get_clock().now()
		dt = (current_time - self.last_time).nanoseconds / 1e9 #Convert to seconds
		self.last_time = current_time

		#Calculate velocities
		if dt > 0:
			linear_velocity = distance / dt
			angular_velocity = delta_theta / dt
		else:
			linear_velocity = 0.0
			angular_velocity = 0.0

		#create odometry message
		odom_msg = Odometry()
		odom_msg.header.stamp = current_time.to_msg()
		odom_msg.header.frame_id = 'odom'
		odom_msg.child_frame_id = 'base_link'

		#set position
		odom_msg.pose.pose.position.x = self.x
		odom_msg.pose.pose.position.y = self.y
		odom_msg.pose.pose.position.z = 0.0

		#set orientation
		odom_msg.pose.pose.orientation.x = 0.0
		odom_msg.pose.pose.orientation.y = 0.0
		odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
		odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

		#set velocity
		odom_msg.twist.twist.linear.x = linear_velocity
		odom_msg.twist.twist.linear.y = 0.0
		odom_msg.twist.twist.angular.z = angular_velocity

		#publish odometry
		self.odom_pub.publish(odom_msg)

		#publish TF transform
		transform = TransformStamped()
		transform.header.stamp = current_time.to_msg()
		transform.header.frame_id = 'odom'
		transform.child_frame_id = 'base_link'

		#set translation (position)
		transform.transform.translation.x = self.x
		transform.transform.translation.y = self.y
		transform.transform.translation.z = 0.0

		#set rotation (orientation)
		transform.transform.rotation.x = 0.0
		transform.transform.rotation.y = 0.0
		transform.transform.rotation.z = math.sin(self.theta / 2.0)
		transform.transform.rotation.w = math.cos(self.theta / 2.0)

		# Broadcast transform
		self.tf_broadcaster.sendTransform(transform)


def main(args=None):
	"""Main entry point"""
	rclpy.init(args=args)
	node = OdometryNode()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info('Shutting down odometry node...')
	finally:
		node.destry_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()


