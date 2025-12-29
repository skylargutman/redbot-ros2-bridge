#!/usr/bin.env python3
"""
Redbot ROS2 Serial Bridge Node
Communicates with Arduino-based Redbot via serial port
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Int32MultiArray
import serial
import time


class RedBotBridge(Node):
	"""ROS2 Node that bridges serial communication with Redbot"""
	def __init__(self):
		#initialize the parent node class with our node name
		super().__init__('redbot_bridge')

		#declare parameters 
		self.declare_parameter('serial_port', '/dev/ttyUSB0')
		self.declare_parameter('baud_rate', 115200)

		#Get parameter values
		port = self.get_parameter('serial_port').value
		baud = self.get_parameter('baud_rate').value

		#connect to arduino
		self.get_logger().info(f'Connecting to {port} at {baud} baud...')
		try:
			self.serial = serial.Serial(port, baud, timeout=0.1)
			time.sleep(2) # Wait for arduino to reset
			self.get_logger().info('Connected to Redbot!')

			#publishers - send sensor data to ROS2 topics
			self.line_pub =  self.create_publisher(Int32MultiArray, 'line_sensors', 10)
			self.accel_pub = self.create_publisher(Int32MultiArray, 'accelerometer', 10)
			self.encoder_pub = self.create_publisher(Int32MultiArray, 'encoders', 10)
			self.bump_left_pub = self.create_publisher(Bool, 'bumper/left', 10)
			self.bump_right_pub = self.create_publisher(Bool, 'bumper/right', 10)

			#publisher for motor commands (for odometry direction correction)
			self.motor_cmd_pub = self.create_publisher(Int32MultiArray, 'motor_commands', 10)

			#subscriber - receive velocity commands from ROS2
			self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

			#store wheelbase for velocity calculations (meters)
			self.wheel_base = 0.135 #distance between wheels
			self.max_speed = 255 #max motor speed value

			#timer - read sensors at 20Hz (every 0.05 seconds)
			self.timer = self.create_timer(0.05, self.read_sensors)

			self.get_logger().info('RedBot bridge Ready!')

		except serial.SerialException as e:
			self.get_logger().error(f'Failed to connect: {e}')
			raise


	def cmd_vel_callback(self, msg):
		"""Convert Twist message to motor speeds and send to Arduino"""

		# Extract linear (forward/backward) and angular (turn) velocities
		linear = msg.linear.x
		angular = msg.angular.z

		#convert to differential drive (left right wheel speeds)
		#when angular is positive, robot turns left (right wheel faster)
		left_speed = linear - (angular * self.wheel_base / 2.0)
		right_speed = linear + (angular * self.wheel_base / 2.0)

		#Scale to motor range (-255 to 255)
		left_motor = int(left_speed * self.max_speed)
		right_motor = int(right_speed * self.max_speed)

		#constrain to valid range
		left_motor = max(-255, min(255, left_motor))
		right_motor = max(-255, min(255, right_motor))

		#send command to Arduino
		command = f"MOTOR,{left_motor},{right_motor}\n"
		self.serial.write(command.encode())

		#publish motor commands for odometry direction correction
		motor_msg = Int32MultiArray()
		motor_msg.data = [left_motor, right_motor]
		self.motor_cmd_pub.publish(motor_msg)


	def read_sensors(self):
		"""Read sensor data from Arduino and publish to ROS2 topics"""
		#check if data is available
		if self.serial.in_waiting == 0:
			return

		try:
			#read a line from Arduino
			line = self.serial.readline().decode().strip()

			#Ignore non-sensor lines (like ACK responses)
			if not line.startswith('SENSORS,'):
				return

			#remove sensors prefix and split by commas
			line = line.replace('\x00', '')  #remove null bytes
			data = line[8:].split(',')

			#check we have all 9 values
			if len(data) != 9:
				return

			#parse values
			line_c = int(data[0]) #line center
			line_r = int(data[1]) #line right
			accel_x = int(data[2]) #Accelerometer X
			accel_y = int(data[3]) #Accelerometer Y
			accel_z = int(data[4]) #Accelerometer Z
			enc_left = int(data[5]) #left encoder
			enc_right = int(data[6]) #right encoder
			bump_l = int(data[7]) #left bumper
			bump_r = int(data[8]) #right bumper

			#publish line sensor data
			line_msg = Int32MultiArray()
			line_msg.data = [line_c, line_r]
			self.line_pub.publish(line_msg)

			#publish accelerometer data
			accel_msg = Int32MultiArray()
			accel_msg.data = [accel_x, accel_y, accel_z]
			self.accel_pub.publish(accel_msg)

			#publish encoder data
			enc_msg = Int32MultiArray()
			enc_msg.data = [enc_left, enc_right]
			self.encoder_pub.publish(enc_msg)

			#publish bumper states
			bump_left_msg = Bool()
			bump_left_msg.data = (bump_l == 1)
			self.bump_left_pub.publish(bump_left_msg)

			bump_right_msg = Bool()
			bump_right_msg.data = (bump_r == 1)
			self.bump_right_pub.publish(bump_right_msg)

		except Exception as e:
			self.get_logger().warn(f'Error reading sensors: {e}')


def main(args=None):
	"""Main entry point"""

	#initialize ROS2
	rclpy.init(args=args)

	#create out node
	node = RedBotBridge()

	try:
		#keep the node running until ctrl+c
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info('Shutting down...')
	finally:
		#clean up
		node.serial.close()
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()



