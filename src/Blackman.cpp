#include "rosneuro_filters/Blackman.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(rosneuro::Blackman<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Blackman<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Blackman<double>, rosneuro::Filter<double>)
