#include <iirob_filters/threshold_filter.h>


ThresholdFilter::ThresholdFilter(double threshold_lin, double threshold_angular) : threshold_lin_(threshold_lin), threshold_angular_(threshold_angular)
{}

geometry_msgs::WrenchStamped ThresholdFilter::applyFilter(const geometry_msgs::WrenchStamped& to_filter_wrench)
{
    geometry_msgs::WrenchStamped filtered_wrench;
    filtered_wrench.header=to_filter_wrench.header;

    if (fabs(to_filter_wrench.wrench.force.x) > threshold_lin_)
    {
        double sign = (to_filter_wrench.wrench.force.x > 0) ? 1 : -1;
        filtered_wrench.wrench.force.x = to_filter_wrench.wrench.force.x-threshold_lin_*sign;
    }
    if (fabs(to_filter_wrench.wrench.force.y) > threshold_lin_)
    {
        double sign = (to_filter_wrench.wrench.force.y > 0) ? 1 : -1;
        filtered_wrench.wrench.force.y = to_filter_wrench.wrench.force.y-threshold_lin_*sign;
    }
    if (fabs(to_filter_wrench.wrench.force.z) > threshold_lin_)
    {
        double sign = (to_filter_wrench.wrench.force.z > 0) ? 1 : -1;
        filtered_wrench.wrench.force.z = to_filter_wrench.wrench.force.z-threshold_lin_*sign;
    }
    if (fabs(to_filter_wrench.wrench.torque.x) > threshold_angular_)
    {
        double sign = (to_filter_wrench.wrench.torque.x > 0) ? 1 : -1;
        filtered_wrench.wrench.torque.x = to_filter_wrench.wrench.torque.x-threshold_angular_*sign;
    }
    if (fabs(to_filter_wrench.wrench.torque.y) > threshold_angular_)
    {
        double sign = (to_filter_wrench.wrench.force.y > 0) ? 1 : -1;
        filtered_wrench.wrench.torque.y = to_filter_wrench.wrench.torque.y-threshold_angular_*sign;
    }
    if (fabs(to_filter_wrench.wrench.torque.z) > threshold_angular_)
    {
        double sign = (to_filter_wrench.wrench.torque.z > 0) ? 1 : -1;
        filtered_wrench.wrench.torque.z = to_filter_wrench.wrench.torque.z-threshold_angular_*sign;
    }

    return filtered_wrench;
}
