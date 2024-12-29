uint16 board_idx # ICS controller ID (2n or 2n+1)

# Pressure control parameters
# If trigger_pressure < target_pressure: pressurization mode
# If trigger_pressure > target_pressure: depressurization mode
float32 trigger_pressure # Pressure threshold to start valve operation [kPa]
float32 target_pressure # Desired pressure level for optimal operation [kPa]

# Air release parameters
# If the value is zero, pressure control is performed based on trigger_pressure and target_pressure
# If the value is greater than zero, solenoid valves opens for release_duration[s] and the work conducts to the atmosphere
float32 release_duration # Duration for atmospheric venting [s]
