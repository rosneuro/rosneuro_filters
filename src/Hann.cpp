#include "rosneuro_filters/Hann.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(rosneuro::Hann<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Hann<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Hann<double>, rosneuro::Filter<double>)

