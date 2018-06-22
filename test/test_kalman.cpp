
#include <iirob_filters/kalman_filter.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <ros/console.h>
#include <log4cxx/logger.h>



// Declare a test
// TEST(TestSuite, testCase1)
// {
// <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

typedef iirob_filters::MultiChannelKalmanFilter<double> KalmanFilter;

TEST(KalmanFilter, testFunctionality)
{
    ROS_ERROR("1");
  KalmanFilter* filter = new KalmanFilter();
  
  // configure default namespace: "KalmanFilter"
  EXPECT_TRUE(filter->configure());
    ROS_ERROR("2");
  
  const int n = 1, m = 1;
  
  const std::vector<double> measurements = {0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41, 0.45};
  const std::vector<double> state_estimates = {0., 0.3545454545, 0.4238095238, 0.4419354839, 0.4048780488, 
    0.3745098039, 0.3655737705, 0.361971831, 0.3765432099, 0.3802197802};
  const std::vector<double> error_cov_estimates = {1., 0.0909090909, 0.0476190476, 0.0322580645, 
    0.0243902439, 0.0196078431, 0.0163934426, 0.014084507, 0.012345679, 0.010989011};
  const std::vector<double> state_updates = {0.3545454545, 0.4238095238, 0.4419354839, 0.4048780488, 
    0.3745098039, 0.3655737705, 0.361971831, 0.3765432099, 0.3802197802, 0.3871287129};
  const std::vector<double> error_cov_updates = {0.0909090909, 0.0476190476, 0.0322580645, 
    0.0243902439, 0.0196078431, 0.0163934426, 0.014084507, 0.012345679, 0.010989011, 0.0099009901};
  const double eps = 0.0001;
  std::vector<double> measurement, state_out;
  Eigen::MatrixXd error_cov_out;
  
  EXPECT_TRUE(filter->getErrorCovarianceMatrix(error_cov_out));
    ROS_ERROR("3");
  EXPECT_TRUE(filter->getCurrentState(state_out));
    ROS_ERROR("31");
  
  for (int i = 0; i < measurements.size(); i++) 
  {
    ROS_ERROR("4, %d", i);
    // configure and check every step  
    if (i != 0)
    {
      EXPECT_TRUE(filter->predict(state_out));
    }
    ROS_ERROR("5, %d", i);
    
    EXPECT_TRUE(state_out.size() == 1);
    ROS_ERROR("6, %d", i);
    EXPECT_NEAR (state_out[0], state_estimates[i], eps);
    ROS_ERROR("7, %d", i);
    
    EXPECT_TRUE(filter->getErrorCovarianceMatrix(error_cov_out));
    ROS_ERROR("8, %d", i);
    EXPECT_TRUE(error_cov_out.size() == 1);
    ROS_ERROR("9, %d", i);
    EXPECT_NEAR (error_cov_out(0, 0), error_cov_estimates[i], eps);
    ROS_ERROR("10, %d", i);
    
    measurement.clear(); measurement.push_back(measurements[i]);
    ROS_ERROR("11, %d", i);
    EXPECT_TRUE(filter->update(measurement, state_out));
    ROS_ERROR("12, %d", i);
    EXPECT_TRUE(state_out.size() == 1);
    ROS_ERROR("13, %d", i);
    EXPECT_NEAR (state_out[0], state_updates[i], eps);
    ROS_ERROR("14, %d", i);
    
    EXPECT_TRUE(filter->getErrorCovarianceMatrix(error_cov_out));
    ROS_ERROR("15, %d", i);
    EXPECT_TRUE(error_cov_out.size() == 1);
    ROS_ERROR("16, %d", i);
    EXPECT_NEAR (error_cov_out(0, 0), error_cov_updates[i], eps);
    ROS_ERROR("17, %d", i);
    
  }
}

  
TEST(KalmanFilter, testParameters)
{
  KalmanFilter* filter = new KalmanFilter();
  
  const std::vector<std::string> namespaces_with_param_errors = {"KalmanFilterTestWithoutParameters", "KalmanFilterTestDt", 
    "KalmanFilterTestN", "KalmanFilterTestN", 
    "KalmanFilterTestParameterA", "KalmanFilterTestParameterC", "KalmanFilterTestParameterQ",
    "KalmanFilterTestParameterR", "KalmanFilterTestParameterP", "KalmanFilterTestParameterX0"
  };
  
  
  for (int i = 0; i < namespaces_with_param_errors.size(); i++)  
  {
    ROS_ERROR("18, %d", i);
    EXPECT_FALSE(filter->configure(namespaces_with_param_errors[i]));
    ROS_ERROR("19, %d", i);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
//   ROSCONSOLE_AUTOINIT;
//   log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//   my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
