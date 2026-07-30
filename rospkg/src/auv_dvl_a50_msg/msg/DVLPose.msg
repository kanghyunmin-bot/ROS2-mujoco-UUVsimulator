# Dead-reckoning pose (intended to be embedded inside DVL.msg)
# ts: unix timestamp in seconds (from report)
float64 ts
# Position in meters (x: forward, y: right, z: down)
float64 x
float64 y
float64 z
# Standard deviation (position uncertainty) in meters
float64 std
# Orientation in degrees
float64 roll
float64 pitch
float64 yaw
# Report status (0 = ok, 1 = issue)
int64 status
# Report format string (e.g. "json_v3.1")
string form