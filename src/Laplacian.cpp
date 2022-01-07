#include "rosneuro_filters/Laplacian.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(rosneuro::Laplacian<int>, rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Laplacian<float>, rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Laplacian<double>, rosneuro::Filter<double>)
