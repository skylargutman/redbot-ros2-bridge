#!/usr/bin/env python3
"""
Launch file for RedBot
Starts the serial bridge and teleop keyboard
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
	return LaunchDescription([
		#start the Redbot bridge node
		Node(
			package='redbot_bridge',
			executable='redbot_node',
			name='redbot_bridge',
			output='screen',
			parameters=[
				{'serial_port': '/dev/ttyUSB0'},
				{'baud_rate': 115200}
			]
		),


		#start teleop keyboard
#		Node(
#			package='teleop_twist_keyboard',
#			executable='teleop_twist_keyboard',
#			name='teleop',
#			output='screen',
#			#prefix='xterm -e'  #opens in a new window
#		),


		#start the odometry node
		Node(
			package='redbot_bridge',
			executable='odometry_node',
			name='odometry',
			output='screen',
			parameters=[
				{'wheel_diameter': 0.065},
				{'wheel_base': 0.15},
				{'ticks_per_revolution': 192},
			],
		),

	])

