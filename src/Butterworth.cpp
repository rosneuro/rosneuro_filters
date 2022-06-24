#include "rosneuro_filters/Butterworth.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(rosneuro::Butterworth<int>, rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Butterworth<float>, rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Butterworth<double>, rosneuro::Filter<double>)
