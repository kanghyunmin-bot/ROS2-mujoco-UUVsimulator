# hit25_auv/msg/YawBias.msg
# Accumulated yaw-bias status (all in radians unless stated).
# Fields are named in English as requested.
#
# bias_rad : Accumulated yaw-bias (EWMA) [rad]
# correction_count : Number of completed corrections
# post_correction_bias_rad : Bias measured right after a correction finishes [rad]
# correcting : 1 = correction in progress, 0 = not correcting


float64 bias_rad
uint32 correction_count
float64 post_correction_bias_rad
uint8 correcting
