# Data for a single polynomial trajectory segment
# Format of data for mth order polynomial:
# [t0 t1 a_0 a_1 ... a_m]
# t0, t1 are start and end times
# a_i are the coefficients of the polynomial
# It is assumed that agents will transform time t
# to be in the interval [0,1]

# Should times be changed to a ROS time format?

float64 t0
float64 t1
float64[] coefficients
