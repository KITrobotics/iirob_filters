#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Trigger.h>

class ThresholdFilter
{

private:
  double threshold_lin_;
  double threshold_angular_;

public:

  geometry_msgs::WrenchStamped applyFilter(const geometry_msgs::WrenchStamped& to_filter_wrench);

  ThresholdFilter(double threshold_lin, double threshold_angular);

};
