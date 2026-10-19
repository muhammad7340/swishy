#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Define color ranges (in HSV) for object detection
            color_ranges = {
                'yellow': ((20, 100, 100), (30, 255, 255)),
                'red': ((0, 120, 70), (10, 255, 255)),
                'blue': ((100, 150, 0), (140, 255, 255)),
                'brown': ((10, 50, 20), (20, 255, 200)),
                'gray': ((90, 0, 50), (120, 50, 200))
            }
            
            # Convert image to HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            for color, (lower, upper) in color_ranges.items():
                mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
                
                # Find contours for detected objects
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                      # Green bounding box
                    cv2.putText(cv_image, '==', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Display the processed image
            cv2.imshow('Object Detection', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
