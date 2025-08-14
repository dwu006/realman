#!/usr/bin/env python3.8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import os
from std_msgs.msg import Header

class CherryYOLONode:
    def __init__(self):
        rospy.init_node('cherry_yolo_detection_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_matrix = None
        self.depth_image = None
        self.color_image = None
        self.camera_info_received = False
        
        # YOLO model setup
        self.model = None
        self.load_yolo_model()
        
        # Detection parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.cherry_class_id = rospy.get_param('~cherry_class_id', 0)
        self.min_cherry_size = rospy.get_param('~min_cherry_size', 0.02)  # meters
        self.max_cherry_size = rospy.get_param('~max_cherry_size', 0.08)  # meters
        self.min_height = rospy.get_param('~min_height', 0.3)  # meters above ground
        self.max_height = rospy.get_param('~max_height', 1.5)  # meters above ground
        
        # Publishers
        self.cherry_pub = rospy.Publisher('/cherry_detection', PointStamped, queue_size=10)
        self.cherry_list_pub = rospy.Publisher('/cherry_list', PointStamped, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/cherry_detection_debug', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        
        # Detection state
        self.detected_cherries = []
        self.last_detection_time = rospy.Time.now()
        self.detection_timeout = rospy.Duration(2.0)  # 2 seconds
        
        # Debug visualization
        self.show_debug = rospy.get_param('~show_debug', True)
        
        rospy.loginfo("CherryYOLONode initialized successfully")
        rospy.loginfo(f"Confidence threshold: {self.confidence_threshold}")
        rospy.loginfo(f"Cherry class ID: {self.cherry_class_id}")
    
    def load_yolo_model(self):
        """Load YOLO model for cherry detection"""
        try:
            # Try to load custom trained model first
            model_path = rospy.get_param('~model_path', 'cherry_detection_model.pt')
            
            if os.path.exists(model_path):
                self.model = YOLO(model_path)
                rospy.loginfo(f"Loaded custom model: {model_path}")
            else:
                # Fall back to pre-trained model
                self.model = YOLO('yolov8n.pt')
                rospy.logwarn(f"Custom model not found at {model_path}, using pre-trained model")
                rospy.logwarn("You should train a custom model for better cherry detection!")
            
            # Set model to evaluation mode
            self.model.eval()
            
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            self.model = None
    
    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.K).reshape(3, 3)
            self.camera_info_received = True
            rospy.loginfo("Camera info received")
    
    def color_callback(self, msg):
        """Process color image for cherry detection"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run detection if we have all required data
            if (self.camera_info_received and 
                self.depth_image is not None and 
                self.model is not None):
                
                self.detect_cherries()
                
        except Exception as e:
            rospy.logerr(f"Error in color callback: {e}")
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"Error in depth callback: {e}")
    
    def detect_cherries(self):
        """Main cherry detection function"""
        if self.color_image is None or self.depth_image is None:
            return
        
        # Run YOLO detection
        results = self.model(self.color_image, verbose=False)
        
        # Process detections
        current_detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Extract detection info
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())
                    
                    # Filter by confidence and class
                    if confidence > self.confidence_threshold and class_id == self.cherry_class_id:
                        # Calculate 3D position
                        cherry_3d = self.get_3d_position(x1, y1, x2, y2)
                        
                        if cherry_3d is not None:
                            # Create detection object
                            detection = {
                                'position': cherry_3d,
                                'confidence': confidence,
                                'bbox': [x1, y1, x2, y2],
                                'size': self.calculate_cherry_size(x1, y1, x2, y2, cherry_3d[2])
                            }
                            
                            # Filter by size and height
                            if self.filter_cherry_detection(detection):
                                current_detections.append(detection)
        
        # Update detected cherries list
        if current_detections:
            self.detected_cherries = current_detections
            self.last_detection_time = rospy.Time.now()
            
            # Publish best cherry (closest to camera)
            best_cherry = self.select_best_cherry(current_detections)
            if best_cherry:
                self.publish_cherry_position(best_cherry)
        
        # Publish debug image
        if self.show_debug:
            self.publish_debug_image(current_detections)
    
    def get_3d_position(self, x1, y1, x2, y2):
        """Convert 2D detection to 3D position using depth"""
        try:
            # Calculate center of bounding box
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)
            
            # Check bounds
            if (0 <= center_y < self.depth_image.shape[0] and 
                0 <= center_x < self.depth_image.shape[1]):
                
                # Get depth at center point (average over small region)
                depth_region = self.depth_image[
                    max(0, center_y-2):min(self.depth_image.shape[0], center_y+3),
                    max(0, center_x-2):min(self.depth_image.shape[1], center_x+3)
                ]
                
                # Filter out invalid depth values
                valid_depths = depth_region[depth_region > 0]
                
                if len(valid_depths) > 0:
                    depth = np.median(valid_depths) / 1000.0  # Convert to meters
                    
                    if 0.1 < depth < 3.0:  # Reasonable depth range
                        # Convert to 3D coordinates
                        point_2d = np.array([center_x, center_y, 1])
                        point_3d = np.linalg.inv(self.camera_matrix) @ point_2d * depth
                        
                        return point_3d
        
        except Exception as e:
            rospy.logwarn(f"Error calculating 3D position: {e}")
        
        return None
    
    def calculate_cherry_size(self, x1, y1, x2, y2, depth):
        """Calculate cherry size in meters"""
        try:
            # Pixel size
            pixel_width = x2 - x1
            pixel_height = y2 - y1
            pixel_size = max(pixel_width, pixel_height)
            
            # Convert to meters using depth and camera parameters
            focal_length = self.camera_matrix[0, 0]  # Assuming fx ≈ fy
            real_size = (pixel_size * depth) / focal_length
            
            return real_size
            
        except Exception as e:
            rospy.logwarn(f"Error calculating cherry size: {e}")
            return 0.0
    
    def filter_cherry_detection(self, detection):
        """Filter cherry detection based on size and position"""
        position = detection['position']
        size = detection['size']
        
        # Filter by size
        if not (self.min_cherry_size <= size <= self.max_cherry_size):
            return False
        
        # Filter by height (assuming camera is mounted above ground)
        if not (self.min_height <= position[2] <= self.max_height):
            return False
        
        # Filter by distance from camera center (avoid edge detections)
        distance_from_center = np.sqrt(position[0]**2 + position[1]**2)
        if distance_from_center > 1.0:  # More than 1m from camera center
            return False
        
        return True
    
    def select_best_cherry(self, detections):
        """Select the best cherry from multiple detections"""
        if not detections:
            return None
        
        # Sort by confidence and distance
        for detection in detections:
            position = detection['position']
            distance = np.sqrt(position[0]**2 + position[1]**2 + position[2]**2)
            detection['distance'] = distance
        
        # Sort by confidence first, then by distance
        sorted_detections = sorted(detections, 
                                 key=lambda x: (x['confidence'], -x['distance']), 
                                 reverse=True)
        
        return sorted_detections[0]
    
    def publish_cherry_position(self, detection):
        """Publish detected cherry position"""
        try:
            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "camera_color_optical_frame"
            msg.point.x = detection['position'][0]
            msg.point.y = detection['position'][1]
            msg.point.z = detection['position'][2]
            
            self.cherry_pub.publish(msg)
            
            rospy.loginfo(f"Cherry detected - Position: [{detection['position'][0]:.3f}, "
                         f"{detection['position'][1]:.3f}, {detection['position'][2]:.3f}], "
                         f"Confidence: {detection['confidence']:.2f}, "
                         f"Size: {detection['size']:.3f}m")
            
        except Exception as e:
            rospy.logerr(f"Error publishing cherry position: {e}")
    
    def publish_debug_image(self, detections):
        """Publish debug image with detections drawn"""
        try:
            if self.color_image is None:
                return
            
            debug_image = self.color_image.copy()
            
            # Draw detections
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                confidence = detection['confidence']
                position = detection['position']
                
                # Draw bounding box
                cv2.rectangle(debug_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                # Draw label
                label = f"Cherry: {confidence:.2f}"
                cv2.putText(debug_image, label, (int(x1), int(y1)-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Draw 3D position
                pos_label = f"({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
                cv2.putText(debug_image, pos_label, (int(x1), int(y2)+20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            
            # Convert back to ROS message
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"Error publishing debug image: {e}")
    
    def get_detection_status(self):
        """Get current detection status"""
        time_since_detection = rospy.Time.now() - self.last_detection_time
        return {
            'cherries_detected': len(self.detected_cherries),
            'last_detection_time': self.last_detection_time,
            'detection_fresh': time_since_detection < self.detection_timeout,
            'camera_ready': self.camera_info_received
        }

def main():
    try:
        node = CherryYOLONode()
        rospy.loginfo("CherryYOLONode started")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("CherryYOLONode interrupted")
    except Exception as e:
        rospy.logerr(f"CherryYOLONode error: {e}")

if __name__ == '__main__':
    main()
