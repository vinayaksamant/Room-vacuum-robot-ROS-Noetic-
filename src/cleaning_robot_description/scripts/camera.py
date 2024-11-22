import rospy
import cv2
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Load the YOLO model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ab2004/yolov5/runs/train/exp7/weights/best.pt')  # path to your trained model

# Initialize the CvBridge class
bridge = CvBridge()

def detect_legs(cv_image):
    # Perform inference
    results = model(cv_image)

    # Parse results
    for det in results.xyxy[0]:
        x1, y1, x2, y2, conf, cls = det
        if conf > 0.3:  # confidence threshold
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(cv_image, f'Leg: {conf:.2f}', (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Show the output image (for debugging purposes)
    cv2.imshow("Output", cv_image)
    cv2.waitKey(3)

def image_callback(msg):
    try:
        # Convert the ROS Image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return
    
    # Detect legs in the image
    detect_legs(cv_image)

def main():
    rospy.init_node('leg_detection_node', anonymous=True)
    rospy.Subscriber('/Camera_1/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()