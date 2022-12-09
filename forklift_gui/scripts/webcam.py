import rospy
import cv2
from sensor_msgs.msg import Image

# Initialize ROS node and publisher
rospy.init_node('webcam_publisher')
pub = rospy.Publisher('webcam_video', Image, queue_size=10)

# Open the webcam and start capturing frames
video_capture = cv2.VideoCapture(0)

while not rospy.is_shutdown():
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    # Convert the frame to a ROS image message
    image_message = Image()
    image_message.header.stamp = rospy.Time.now()
    image_message.data = cv2.imencode('.jpg', frame)[1].tobytes()

    # Publish the image message
    pub.publish(image_message)

# Release the webcam
video_capture.release()
