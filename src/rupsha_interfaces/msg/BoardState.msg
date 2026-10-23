# What the board tells the Pi, at 50 Hz. See firmware-contract.md.
# encoder_ticks: 1873 per metre of drive-axle travel, signed, forward positive.
# orientation: BNO085 gyro-integrated rotation vector, not an Euler triple.
# gps_fix: 0 none, 3 a 3D fix. The sim reports 3 when the navsat sample is finite.
std_msgs/Header header
int32 encoder_ticks
geometry_msgs/Quaternion orientation
float64 latitude_deg
float64 longitude_deg
float32 altitude_m
uint8 gps_fix
float32 battery_v
bool link_fresh
