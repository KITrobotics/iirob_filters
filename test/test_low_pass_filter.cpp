/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021 PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Boston Cleek
   Desc:   test gravity_compensation
*/

#include <gtest/gtest.h>
#include <thread>
#include <iirob_filters/low_pass_filter.h>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_low_pass_filter");

class LowPassFilterFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    low_pass_filter_.setNode(node_);

    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  LowPassFilterFixture()
    : node_(std::make_shared<rclcpp::Node>("test_low_pass_filter"))
    , executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
  {
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  iirob_filters::LowPassFilter<geometry_msgs::msg::WrenchStamped> low_pass_filter_;
  std::thread executor_thread_;
};

TEST_F(LowPassFilterFixture, TestLowPassFilter)
{
  SCOPED_TRACE("TestLowPassFilter");

  node_->declare_parameter("sampling_frequency", 10.0);
  node_->declare_parameter("damping_frequency", 1.0);
  node_->declare_parameter("damping_intensity", 1.0);
  node_->declare_parameter("divider", 1);

  ASSERT_TRUE(low_pass_filter_.configure());

  geometry_msgs::msg::WrenchStamped in, out;
  in.header.frame_id = "world";
  in.wrench.force.x = 1.0;
  in.wrench.torque.x = 10.0;

  ASSERT_TRUE(low_pass_filter_.update(in, out));

  ASSERT_EQ(out.wrench.force.x, 0.0);
  ASSERT_EQ(out.wrench.force.y, 0.0);
  ASSERT_EQ(out.wrench.force.z, 0.0);

  ASSERT_EQ(out.wrench.torque.x, 0.0);
  ASSERT_EQ(out.wrench.torque.y, 0.0);
  ASSERT_EQ(out.wrench.torque.z, 0.0);

  ASSERT_TRUE(low_pass_filter_.update(in, out));

  EXPECT_NEAR(out.wrench.force.x, 0.546612, 1e-3);
  ASSERT_EQ(out.wrench.force.y, 0.0);
  ASSERT_EQ(out.wrench.force.z, 0.0);

  EXPECT_NEAR(out.wrench.torque.x, 5.466116, 1e-3);
  ASSERT_EQ(out.wrench.torque.y, 0.0);
  ASSERT_EQ(out.wrench.torque.z, 0.0);
}

int main(int argc, char** argv)
{
  // It is important we init ros before google test because we are going to
  // create a node_ durring the google test init.
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
