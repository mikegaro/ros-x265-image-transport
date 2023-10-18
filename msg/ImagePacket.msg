Header  header          # Original sensor_msgs/Image header
uint8[] data            # ffmpeg compressed payload
int32   width           # original image width
int32   height          # original image height
string  pixFormat       # Specifies pixel format
uint64  pts             # packet pts
uint8  flags            # packet flags
