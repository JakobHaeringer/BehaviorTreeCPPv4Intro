##
# @file EgoPose.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for the current egoPose on the global map.
# @param nodeID        NodeID on the global map.
# @param x             X coordinate on the global map.
# @param y             Y coordinate on the global map.
# @param yaw           Yaw angle on the global map.
# @version 1.0
# @date 2023-07-13
uint8 nodeID
float32 x
float32 y
float32 yaw