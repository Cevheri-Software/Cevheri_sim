#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import threading

# Import YOLOv8 (uncomment when actually using YOLOv8)
# from ultralytics import YOLO

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')
        self.get_logger().info('Object Tracking Node Started')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers for RGB and depth images
        self.rgb_sub = self.create_subscription(
            Image, 
            '/camera', 
            self.rgb_callback, 
            10)
            
        self.depth_sub = self.create_subscription(
            Image, 
            '/depth_camera', 
            self.depth_callback, 
            10)
            
        # Create publishers for processed images and detections
        self.detection_pub = self.create_publisher(
            Image,
            '/object_tracking/detection_image',
            10)
            
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/object_tracking/detection_markers',
            10)
            
        # Initialize local variables
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.processing = False
        self.lock = threading.Lock()
        
        # Initialize YOLOv8 model (uncomment when actually using YOLOv8)
        # self.model = YOLO('yolov8n.pt')  # Load a pretrained YOLOv8 model
        
        # Create a timer to process images
        self.create_timer(0.1, self.process_images)
        
        self.get_logger().info('Object Tracking Node initialized')
        
    def rgb_callback(self, msg):
        """
        Callback for RGB camera images
        """
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            
    def depth_callback(self, msg):
        """
        Callback for depth camera images
        """
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            
    def process_images(self):
        """
        Process the latest RGB and depth images for object detection
        """
        # Check if we have valid images and we're not already processing
        if self.latest_rgb_image is None or self.latest_depth_image is None or self.processing:
            return
            
        # Set processing flag
        self.processing = True
        
        # Make a copy of the current images to avoid race conditions
        with self.lock:
            rgb_image = self.latest_rgb_image.copy()
            depth_image = self.latest_depth_image.copy()
            
        # Example of simple color-based object detection (red box)
        # Replace this with YOLOv8 detection when available
        self.detect_objects_by_color(rgb_image, depth_image)
        
        # Another option - uncomment to use YOLOv8 instead of color detection
        # self.detect_objects_with_yolo(rgb_image, depth_image)
        
        # Reset processing flag
        self.processing = False
        
    def detect_objects_by_color(self, rgb_image, depth_image):
        """
        Detect objects by color (simple example - will detect red objects)
        """
        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color detection
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Initialize marker array
        marker_array = MarkerArray()
        
        # Process detected contours
        for i, contour in enumerate(contours):
            # Filter by contour area
            if cv2.contourArea(contour) < 500:  # Minimum area threshold
                continue
                
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Draw rectangle on image
            cv2.rectangle(rgb_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(rgb_image, f'Object {i}', (x, y-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Get depth at center of object
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Ensure the center point is within the depth image bounds
            if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                # Get depth value (in meters)
                depth_value = depth_image[center_y, center_x]
                
                # Create a marker for visualization
                marker = self.create_marker(i, center_x, center_y, depth_value)
                marker_array.markers.append(marker)
                
                # Add depth information to the image
                cv2.putText(rgb_image, f'Depth: {depth_value:.2f}m', 
                          (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Publish the processed image
        try:
            detection_msg = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
            self.detection_pub.publish(detection_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        
        # Publish markers
        self.marker_pub.publish(marker_array)
    
    def detect_objects_with_yolo(self, rgb_image, depth_image):
        """
        Detect objects using YOLOv8
        Uncomment and implement when ready to use YOLOv8
        """
        # # Run YOLO detection
        # results = self.model(rgb_image)
        
        # # Initialize marker array
        # marker_array = MarkerArray()
        
        # # Process detected objects
        # for i, (box, score, cls) in enumerate(zip(results[0].boxes.xyxy, 
        #                                          results[0].boxes.conf, 
        #                                          results[0].boxes.cls)):
        #     # Get coordinates
        #     x1, y1, x2, y2 = box.tolist()
        #     x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
        #     # Get object center
        #     center_x = (x1 + x2) // 2
        #     center_y = (y1 + y2) // 2
            
        #     # Get class name
        #     class_name = results[0].names[int(cls)]
            
        #     # Draw rectangle on image
        #     cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #     cv2.putText(rgb_image, f'{class_name} {score:.2f}', (x1, y1-10), 
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        #     # Get depth at center of object
        #     if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
        #         # Get depth value (in meters)
        #         depth_value = depth_image[center_y, center_x]
                
        #         # Create a marker for visualization
        #         marker = self.create_marker(i, center_x, center_y, depth_value)
        #         marker_array.markers.append(marker)
                
        #         # Add depth information to the image
        #         cv2.putText(rgb_image, f'Depth: {depth_value:.2f}m', 
        #                   (x1, y2+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # # Publish the processed image
        # try:
        #     detection_msg = self.bridge.cv2_to_imgmsg(rgb_image, "bgr8")
        #     self.detection_pub.publish(detection_msg)
        # except CvBridgeError as e:
        #     self.get_logger().error(f'CV Bridge error: {e}')
        
        # # Publish markers
        # self.marker_pub.publish(marker_array)
        pass
    
    def create_marker(self, marker_id, image_x, image_y, depth):
        """
        Create a visualization marker for a detected object
        """
        marker = Marker()
        marker.header.frame_id = "map"  # Change to camera frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detection"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set marker position based on camera parameters and depth
        # This is a simplified version - in a real application, you would use proper camera intrinsics
        # and coordinate transformations
        # For now, we'll just use image coordinates and depth as a simplification
        marker.pose.position.x = depth  # Forward
        marker.pose.position.y = (image_x - 320) * depth / 500.0  # Left/Right
        marker.pose.position.z = (240 - image_y) * depth / 500.0  # Up/Down
        
        # Set marker orientation (identity quaternion)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set marker scale
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Set marker color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
