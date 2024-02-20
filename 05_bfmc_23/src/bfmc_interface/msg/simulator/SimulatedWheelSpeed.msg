##
# @file SimulatedWheelSpeed.msg
# @author Constantin Blessing
# @brief This message type is used for the simulated wheel speed sensor data created by the Unity simulator.
# @param total_wheel_ticks_left        Total wheel ticks (left wheel) counted from the start of the Unity simulation.
# @param total_wheel_ticks_right       Total wheel ticks (right wheel) counted from the start of the Unity simulation.
# @version 1.0
# @date 2023-07-13
uint32 total_wheel_ticks_left
uint32 total_wheel_ticks_right