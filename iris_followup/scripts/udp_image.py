import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize ROS node and publisher
rospy.init_node('udp_image')
image_pub = rospy.Publisher('/iris/image/udp', Image, queue_size=10)
bridge = CvBridge()

cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print('Error: Unable to open pipeline')
    exit()

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        print('Error: Unable to read frame')
        break

    ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    image_pub.publish(ros_image)

cap.release()
cv2.destroyAllWindows()
