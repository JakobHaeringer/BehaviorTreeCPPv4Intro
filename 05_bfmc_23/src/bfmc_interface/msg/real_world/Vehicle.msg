##
# @file Vehicle.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for the position sent by other vehicles - V2V communication.
# @param ID        ID of the vehicle.
# @param timestamp Timestamp of the sent vehicle position.
# @param posA      X coordinate of the other vehicle position.
# @param posB      Y coordinate of the other vehicle position.
# @param rotA      not used
# @param rotB      not used
# @version 1.0
# @date 2023-07-13
uint8 ID
float32 timestamp
float32 posA
float32 posB
float32 rotA
float32 rotB