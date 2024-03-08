std_msgs/Header header
uint8 dim # dimension of polynomial traajectory
float64 tf # transfer duration started from time stamp in header
# coefficients of polynomial traajectory ranged from c0 to cn
std_msgs/Float64[] x_coeff
std_msgs/Float64[] y_coeff
std_msgs/Float64[] z_coeff
std_msgs/Float64[] yaw_coeff