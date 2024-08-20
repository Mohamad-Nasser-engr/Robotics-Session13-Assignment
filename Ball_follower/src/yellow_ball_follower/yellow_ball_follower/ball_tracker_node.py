import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class BallTrackerNode(Node):

    def __init__(self):
        super().__init__('ball_tracker_node')

        # Subscriber to the image topic
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for robot's velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Initialize Twist message
        self.twist = Twist()

        # Flag to check if the ball was just detected
        self.ball_detected = False

    def image_callback(self, data):
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range for the color yellow
        lower_yellow = np.array([25, 150, 150])
        upper_yellow = np.array([35, 255, 255])

        # Create a binary mask where yellow colors are white
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour which should be the ball
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            radius = int(radius)

            # Draw a circle around the detected ball
            cv2.circle(cv_image, center, radius, (0, 255, 0), 2)

            # Set a threshold for how close the ball needs to be to the center of the image
            center_threshold = 20  # Pixels

            if not self.ball_detected:
                # Stop the robot for 1 second when the ball is first detected
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher_.publish(self.twist)
                time.sleep(1)  # Pause for 1 second
                self.ball_detected = True

            if abs(center[0] - cv_image.shape[1] / 2) > center_threshold:
                # Adjust rotation to center the ball
                self.twist.angular.z = -0.002 * (center[0] - cv_image.shape[1] / 2)
            else:
                self.twist.angular.z = 0.0  # Stop rotating if the ball is centered

            if radius > 10:  # Adjust threshold for how close the ball should be
                self.twist.linear.x = 0.2  # Move forward
            else:
                self.twist.linear.x = 0.0  # Stop moving forward if the ball is too close
        else:
            # If no ball is detected, rotate to search for it
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.5  # Rotate to search for the ball
            self.ball_detected = False  # Reset the flag if the ball is lost

        # Publish the velocity command
        self.publisher_.publish(self.twist)

        # Display the image and the mask
        cv2.imshow("Yellow Ball Tracking", cv_image)
        cv2.imshow("Masked Image", mask)
        #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
