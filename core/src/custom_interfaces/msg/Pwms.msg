# The below gives you timestamps. I am leaving it out for now
# std_msgs/Header Header

# f = front / r = rear
# l = left  / r = right
# t = top   / b = bottom
# For example, flt, means front-left-top in order of apperance
# pwm for the thruster of the robot
# A array of 8 ints which maps the thrusters in this order:
# flt, frt, rlt, rrt,
# flb, frb, rlb, rrb

int32[8] pwms