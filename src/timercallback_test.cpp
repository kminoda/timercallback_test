// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <vector>
#include <cmath>
#include <cassert>
#include <filesystem>
#include <sys/stat.h>

#include "rclcpp/rclcpp.hpp"

class TimerCallbackTest : public rclcpp::Node
{
public:
  TimerCallbackTest(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  void timerCallbackDummy();
  rclcpp::TimerBase::SharedPtr timer_dummy_;
};

TimerCallbackTest::TimerCallbackTest(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0));
  std::function<void()> cb =
    std::bind(&TimerCallbackTest::timerCallbackDummy, this);

  // This fails at runtime
  timer_dummy_ = rclcpp::create_timer(
    this, get_clock(), period_ns, cb);

  // This works fine
  timer_dummy_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&TimerCallbackTest::timerCallbackDummy, this));
}

void TimerCallbackTest::timerCallbackDummy(){std::cout << "called" << std::endl;}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<TimerCallbackTest>("timer_callback_test", node_options);

  rclcpp::spin(node);

  return 0;
}
