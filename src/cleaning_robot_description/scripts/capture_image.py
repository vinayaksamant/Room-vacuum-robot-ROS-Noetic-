import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageCapture:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_capture_node', anonymous=True)

        # Create a CvBridge object to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the /Camera_1/image_raw topic
        self.image_sub = rospy.Subscriber('/Camera_1/image_raw', Image, self.image_callback)

        # Initialize OpenCV window
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

        # Image placeholder
        self.cv_image = None

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def run(self):
        img_count = 0
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                # Display the image
                cv2.imshow("Camera Feed", self.cv_image)

                # Wait for key press
                key = cv2.waitKey(1) & 0xFF

                # If 'c' key is pressed, capture the image
                if key == ord('c'):
                    img_name = f"captured_image_{img_count}.jpg"
                    cv2.imwrite(img_name, self.cv_image)
                    print(f"Image saved as {img_name}")
                    img_count += 1

                # Exit on pressing 'q'
                elif key == ord('q'):
                    break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    ic = ImageCapture()
    ic.run()