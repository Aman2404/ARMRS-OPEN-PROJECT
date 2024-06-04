#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep

class RobotMover(Node):
    def __init__(self, robot_id, yaw_rate):
        super().__init__(f'move_robot_{robot_id}')
        self.pub = self.create_publisher(Twist, f'/robot{robot_id}/cmd_vel', 10)
        twist = Twist()
        twist.angular.z = yaw_rate
        self.pub.publish(twist)
        sleep(1)

def main():
    rclpy.init()
    executor = rclpy.executors.MultiThreadedExecutor()

    number_of_robots = 3  # Set this to the number of robots you have
    for robot_id in range(1, number_of_robots + 1):
        robot_mover = RobotMover(robot_id, 0.5)  # Replace 0.5 with the desired yaw rate
        executor.add_node(robot_mover)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    # Shutdown and remove nodes
    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()