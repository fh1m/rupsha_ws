# Everything one frame produced. header.stamp is the frame's capture time, not
# the publish time, so a consumer can measure how stale it is.
std_msgs/Header header
uint32 image_width
uint32 image_height
float32 latency_ms      # capture -> publish
Detection[] detections
