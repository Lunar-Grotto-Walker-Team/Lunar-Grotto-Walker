# Humble 机械臂协调模块完成相机2就位后发送 START；本包不实现机械臂运动。
uint8 START=1
uint8 CANCEL=2

std_msgs/Header header
uint64 request_id
uint8 command
string status_text

