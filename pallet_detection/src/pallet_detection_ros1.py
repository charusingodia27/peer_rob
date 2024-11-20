#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class PalletDetection:

    def __init__(self):
        # Set up parameters
        color_image_topic = '/image_raw'
        output_image_topic = '/pallet_detection/output_image'
        
        # Initialize ROS node components
        self.bridge = CvBridge()
        self.model = YOLO('trained_models/best.pt')
        
        # Set up ROS publishers and subscribers
        rospy.Subscriber(color_image_topic, msg_Image, self.imageCallback)
        self.image_pub = rospy.Publisher(output_image_topic, msg_Image, queue_size=1)

    def imageCallback(self, img_msg):
        try:
            # Convert ROS Image message to OpenCV format
            img_input = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            img_rgb = cv2.cvtColor(img_input, cv2.COLOR_BGR2RGB)

            # Perform YOLO inference
            results = self.model.predict(img_rgb, conf=0.2)

            # Draw bounding boxes on the image
            for detection in results[0].boxes.data:
                x1, y1, x2, y2, conf, cls = detection.tolist()
                cv2.rectangle(img_input, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(img_input, f"Class: {int(cls)} Conf: {conf:.2f}", 
                            (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Convert back to ROS Image message and publish
            output_msg = self.bridge.cv2_to_imgmsg(img_input, encoding='bgr8')
            self.image_pub.publish(output_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

def main():
    rospy.init_node("PalletDetection", anonymous=False)
    PalletDetection()
    rospy.spin()

if __name__ == "__main__":
    main()