# What the Pi is allowed to ask the board for. See firmware-contract.md.
# A heartbeat that stops changing is a dead link: the board zeros speed
# and centres the steer. It does not keep a mission running.
std_msgs/Header header
float32 speed_mms
float32 steer_rad
uint16 camera_pulse_us
uint32 heartbeat
