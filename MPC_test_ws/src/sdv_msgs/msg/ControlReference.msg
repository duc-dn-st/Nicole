# ControlReference.msg
# message which sends the reference for the low level controller

# velocity in m/s
float32 velocity_ref_front_left
float32 velocity_ref_front_right
float32 velocity_ref_back_left
float32 velocity_ref_back_right
# steering angle in rad from -pi to pi
float32 steering_ref_front_left
float32 steering_ref_front_right
float32 steering_ref_back_left
float32 steering_ref_back_right
# boolean if emergency stop necessary
bool stop_motors
