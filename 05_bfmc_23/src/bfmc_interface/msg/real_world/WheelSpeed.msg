##
# @file WheelSpeed.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used for data sent by the wheelspeed sensors.
# @param wheelspeedLeft    [m/s] Current velocity of the left wheel.
# @param wheelspeedRight   [m/s] Current velocity of the right wheel.
# @param velocity          [m/s] Current velocity of the vehicle.
# @param totalDistance     [m] Overall driven distance of the vehicle.
# @version 1.0
# @date 2023-07-13
float32 wheelspeedLeft
float32 wheelspeedRight
float32 velocity
float32 totalDistance