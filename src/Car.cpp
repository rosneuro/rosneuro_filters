#include "rosneuro_filters/Car.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(rosneuro::Car<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Car<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Car<double>, rosneuro::Filter<double>)

