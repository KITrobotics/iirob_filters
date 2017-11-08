#ifndef IIROB_FILTERS_INTERFACE_H
#define IIROB_FILTERS_INTERFACE_H

namespace iirob_filters
{
  class FilterInterface
  {
  public:
    virtual bool init(const ros::NodeHandle &nh) = 0;
    virtual ~FilterInterface(){}

  protected:
    FilterInterface(){}
  };
};
#endif
