# One thing seen in one frame. Pixel-native first: u/v/bearing are always set;
# range and pose only when a metric solve exists (calibrated camera + known size).
string  source          # "aruco" | detector model stem
string  label           # "tag" | class name
int32   tag_id          # ArUco id; -1 for non-tags
float32 confidence      # 0..1; 1.0 for a decoded tag
float32 u               # box centre, px from the left
float32 v               # box centre, px from the top
float32 width_px
float32 height_px
float32 bearing_deg     # from the optical axis, positive = right
float32 range_m         # NaN when no metric solve
geometry_msgs/Pose pose # camera frame (REP-103 optical); valid only when range_m is finite
float32 pose_ambiguity  # IPPE reprojection-error ratio best/second; ~1 = flip-ambiguous
