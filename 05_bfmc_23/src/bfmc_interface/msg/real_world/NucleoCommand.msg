##
# @file NucleoCommand.msg
# @author Jakob Haeringer and Noah Koehler
# @brief This message type is used to send steering/velocity commands to the serialHandlerNODE.
# @param command_type  Type of the command.
# @param value         Value of the specified command.
# @version 1.0
# @date 2023-07-13
uint8 command_type
float32 value