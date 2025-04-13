#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
import time

from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from puppy_control.msg import Pose, Gait, Velocity

# ROS Node Name
ROS_NODE_NAME = "move_image_processing_node"

# Messages
pose_msg = Pose(roll=math.radians(0), pitch=math.radians(0), yaw=0, height=-10, x_shift=0, stance_x=0, stance_y=0, run_time=500)
gait_msg = Gait(overlap_time=0.8, swing_time=0.2, clearance_time=0.2, z_clearance=3)
vel_msg = Velocity(x=0, y=0, yaw_rate=math.radians(0))

# Camera Calibration (adjust based on your setup)
KNOWN_WIDTH = 3.0  # Known width of the object in cm
FOCAL_LENGTH = 600  # Focal length of the camera in pixels

# Globals
target_coordinates = None
distance_to_target = None
image_center = (320, 240)  # Assuming 640x480 resolution
CENTER_TOLERANCE = 20

# Functions
def getContour(img):
    """Find the largest red object in the image and calculate its center and distance."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define red color range
    lower_bright_red = np.array([0, 150, 150])
    upper_bright_red = np.array([10, 255, 255])
    lower_bright_red2 = np.array([170, 150, 150])
    upper_bright_red2 = np.array([180, 255, 255])

    # Create masks
    mask1 = cv2.inRange(hsv, lower_bright_red, upper_bright_red)
    mask2 = cv2.inRange(hsv, lower_bright_red2, upper_bright_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Find contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours
    min_area = 500
    max_area = 0
    best_contour = None

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_area and area > max_area:
            max_area = area
            best_contour = contour

    if best_contour is not None:
        x, y, w, h = cv2.boundingRect(best_contour)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(img, (x + w // 2, y + h // 2), 5, (0, 0, 255), -1)
        center_point = (x + w // 2, y + h // 2)

        # Calculate distance using the pinhole camera model
        distance = (KNOWN_WIDTH * FOCAL_LENGTH) / w
        return img, center_point, distance

    return img, None, None

def start_camera(img):
    """Process camera images to detect the target."""
    global target_coordinates, distance_to_target

    # Convert ROS Image to OpenCV format
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Resize to standard 640x480 resolution
    resized_frame = cv2.resize(cv2_img, (640, 480))

    # Process frame and extract target details
    processed_frame, coordinates, distance = getContour(resized_frame)
    target_coordinates = coordinates
    distance_to_target = distance

    # Display the processed frame
    cv2.imshow("Frame", processed_frame)
    cv2.waitKey(1)

def cleanup():
    """Cleanup resources on shutdown."""
    rospy.loginfo("Shutting down...")
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    cv2.destroyWindow("Frame")

# Main Code
if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)

    # Subscribers and Publishers
    rospy.Subscriber("/usb_cam/image_raw", Image, start_camera)
    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    gait_pub = rospy.Publisher("/puppy_control/gait", Gait, queue_size=1)
    vel_pub = rospy.Publisher("/puppy_control/velocity", Velocity, queue_size=1)

    # Send initial gait message
    gait_pub.publish(gait_msg)
    time.sleep(0.2)

    noTargetDetected = False

    while not rospy.is_shutdown():
        if target_coordinates is not None:
            noTargetDetected = False

            x_error = target_coordinates[0] - image_center[0]
            y_error = target_coordinates[1] - image_center[1]

            if distance_to_target <= 20:
                # Stop movement and center on both axes
                vel_msg.x = 0
                if abs(x_error) > CENTER_TOLERANCE:
                    vel_msg.yaw_rate = -math.radians(x_error / 320 * 30)
                    
                else:
                    vel_msg.yaw_rate = 0
                
                if abs(y_error) > CENTER_TOLERANCE:
                    pose_msg.pitch = -math.radians(y_error / 240 *15)
                else:
                    pose_msg.pitch = 0

                rospy.loginfo("Object too close. Centering...")
            else:
                # Move forward and adjust alignment
                vel_msg.x = 10  # Forward movement
                vel_msg.yaw_rate = -math.radians(x_error / 320 * 30)
                pose_msg.pitch = 0

            rospy.loginfo(f"Target: {target_coordinates}, Distance: {distance_to_target:.2f} cm, X Error: {x_error}, Y Error: {y_error}")
        else:
            # No target detected; stop movement
            vel_msg.x = 0
            vel_msg.yaw_rate = 0

            if not noTargetDetected:
                rospy.loginfo("No target detected.")
            noTargetDetected = True

        vel_pub.publish(vel_msg)
        time.sleep(1.0)  # Smooth updates
        pose_pub.publish(pose_msg)
        time.sleep(1.0)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
