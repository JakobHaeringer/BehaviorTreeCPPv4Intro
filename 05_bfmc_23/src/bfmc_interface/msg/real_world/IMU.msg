##
# @file IMU.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for the IMU data.
# @param roll              [°]    Roll angle
# @param pitch             [°]    Pitch angle
# @param yaw               [°]    Yaw angle
# @param acceleration_x    [m/s²] Acceleration around x-axis
# @param acceleration_y    [m/s²] Acceleration around y-axis
# @param acceleration_z    [m/s²] Acceleration around z-axis
# @version 1.0
# @date 2023-07-13
float32 roll
float32 pitch
float32 yaw
float32 acceleration_x
float32 acceleration_y
float32 acceleration_z