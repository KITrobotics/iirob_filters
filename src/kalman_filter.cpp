#include <iirob_filters/kalman_filter.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(iirob_filters::MultiChannelKalmanFilter<double>, filters::MultiChannelFilterBase<double>)