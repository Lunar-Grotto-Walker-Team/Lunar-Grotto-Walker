# 相机2二次识别结果。三维坐标属于 header.frame_id（相机2彩色光学帧）。
uint8 CONFIRMED=1
uint8 NO_TARGET=2
uint8 TIMEOUT=3
uint8 CANCELLED=4
uint8 ERROR=5

std_msgs/Header header
uint64 request_id
uint8 status
string status_text
uint32 image_width
uint32 image_height
YoloeDetection3D[] detections




