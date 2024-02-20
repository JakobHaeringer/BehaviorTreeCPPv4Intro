##
# @file SimulatedCamera.msg
# @author Constantin Blessing
# @brief This message type is used for the color and depth frame created by the Unity simulator.
# @param color_frame        RGB color frame.
# @param depth_frame        Depth image.
# @version 1.0
# @date 2023-07-13
sensor_msgs/CompressedImage color_frame
sensor_msgs/CompressedImage depth_frame