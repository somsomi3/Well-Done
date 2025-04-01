# LaserScan 데이터 + pose(x, y, theta) 정보를 포함하는 메시지
std_msgs/Header header
float32[] ranges
float32[] intensities

float32 angle_min
float32 angle_max
float32 angle_increment
float32 range_min
float32 range_max

float32 scan_time
float32 time_increment

# 추가 pose 정보
float32 pose_x
float32 pose_y
float32 pose_theta
