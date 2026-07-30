#Header
std_msgs/Header header

# Water Linked position_local / serial wrp timestamp [s]
float64 time

# DVL internal dead-reckoning position [m]
# Axes follow the Water Linked vehicle frame at DR reset:
# x forward, y right/starboard, z down.
geometry_msgs/Vector3 position

# Position standard deviation / FOM [m]
float64 pos_std

# DVL internal DR orientation [deg]: roll about x, pitch about y, yaw about z.
float64 roll
float64 pitch
float64 yaw

# Message type, expected "position_local" for TCP JSON DR.
string type

# Status: 0 means no issue in the Water Linked protocol.
int64 status

# Formatting/protocol version, e.g. "json_v3.3".
string format
