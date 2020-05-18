# SimpleMSRPA Message with a noise boolean
# For convenience of command line usage
#
# Entries of data vector:
# 
# Entries 0, 1 are t0, t1 respectively
# Remaining entries are in this order:
#   linear      : p0, p1
#   polynomial  : a_0, ..., a_l
#   circular    : R, theta0, w, cx, cy, cz (Rovers will ignore cz) 

string trajectory_type
float64[] data
bool noise
float64 mean
float64 variance
