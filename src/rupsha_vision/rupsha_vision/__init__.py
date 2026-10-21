"""Rupsha vision: ArUco posts and YOLO objects -> /rupsha/detections.

Every module here except `vision_node` is ROS-free and importable with nothing
but numpy and cv2, so the truth tests run on any box. `vision_node` imports
rclpy and rupsha_interfaces inside itself only.
"""
