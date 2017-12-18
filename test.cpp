#include <iirob_filters/kalman_filter.h>



iirob_filters::MultiChannelKalmanFilter<double>* filter;

filter = new iirob_filters::MultiChannelKalmanFilter<double>();
bool configure_result = filter->configure();

std::vector<double> in, out;
bool update_result = filter->update(in, out);
