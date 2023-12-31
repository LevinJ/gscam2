import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import sensor_msgs
import cv2
from cv_bridge import CvBridge
import sys

class MinimalSubscriber(Node):

    def __init__(self, input_topic):
        super().__init__('minimal_subscriber')
        self.subscription2 = self.create_subscription(
            sensor_msgs.msg.CompressedImage,
            input_topic,
            self.listener_callback2,
            10)
        self.input_topic = input_topic
        self.subscription2  # prevent unused variable warning
        return

    def listener_callback2(self, msg):
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv2.imshow(self.input_topic,cv_image[...,::-1] )
        cv2.waitKey(1)
        # cv2.imwrite("/home/levin/temp/2.jpg", cv_image)
        return



def main(args=None):
    rclpy.init(args=args)
    input_topic = '/my_camera/image_raw/compressed'
    if len(sys.argv) > 1:
        input_topic = sys.argv[1]

    
    minimal_subscriber = MinimalSubscriber(input_topic)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()