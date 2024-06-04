import os
import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


class ImageSaver(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_image_subscriber_node')
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            Image,
            f'/{robot_name}/camera/image_raw',  
            self.image_callback,
            qos_profile
        )
        self.cv_bridge = CvBridge()
        self.image_count = 0
        self.save_interval = 5 # Save every 3rd image
        self.robot_name = robot_name
        self.image_folder = os.path.expanduser(f'~/Desktop/ARMRS_OPEN_WS/src/my_package/scripts/mapping')


        # Create folder if it doesn't exist
        if not os.path.exists(self.image_folder):
            os.makedirs(self.image_folder)

    def image_callback(self, msg):
        print('Received an image')
        try:
            if(self.image_count < 350):
                if self.image_count % self.save_interval == 0:  # Save every 3rd image
                    cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    image_filename = os.path.join(self.image_folder, f'image_{self.robot_name}_{self.image_count}.jpg')
                    cv2.imwrite(image_filename, cv_image)
                    self.get_logger().info(f'Saved image {image_filename}')
                self.image_count += 1
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    image_saver_robot1 = ImageSaver('robot1')
    image_saver_robot2 = ImageSaver('robot2')
    image_saver_robot3 = ImageSaver('robot3')
    # Add more robots as needed
    print('Started image saver nodes')

    executor.add_node(image_saver_robot1)
    executor.add_node(image_saver_robot2)
    executor.add_node(image_saver_robot3)
    # Remember to spin for additional robots

    print('Started image saver nodes')
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


    image_saver_robot1.destroy_node()
    image_saver_robot2.destroy_node()
    image_saver_robot3.destroy_node()
    # Remember to destroy nodes for additional robots

    rclpy.shutdown()

if __name__ == '__main__':
    main()
