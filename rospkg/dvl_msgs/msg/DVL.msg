#Header
std_msgs/Header header

# Milliseconds since last velocity report (ms)
float64 time

#Measured velocity [m/s]
geometry_msgs/Vector3 velocity

#Figure of Merit
float64 fom

#Covariance matrix, for the velocities. The figure of merit is calculated from this (entries in (m/s)^2)
float64[] covariance

#Altitude of the sensor
float64 altitude

#Beams/Transducers
DVLBeam[] beams

#Validity of velocity
bool velocity_valid

#Status message
int64 status

# Timestamp of the surface reflection, aka 'center of ping' (Unix timestamp in microseconds)
int64 time_of_validity

# Timestamp from immediately before sending of the report over TCP (Unix timestamp in microseconds)
int64 time_of_transmission

#Formatting of json
string form
