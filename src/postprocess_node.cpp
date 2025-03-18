/************************************************************************
Copyright 2025 RoboSense Technology Co., Ltd

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
************************************************************************/
#include "postprocess_node.hpp"

std::string ParseConfigOption(int argc, char* argv[]) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config") {
      if (i + 1 < argc) {
        return argv[i + 1];
      } else {
        std::cerr << "--config need a config file" << std::endl;
        return "";
      }
    }
  }
  return "config/usr_config.yaml";
}

int main(int argc, char** argv) {
  const std::string cfg_path = ParseConfigOption(argc, argv);
  YAML::Node cfg = YAML::LoadFile(cfg_path)["postprocess_node"];

  rclcpp::init(argc, argv);
  auto node = std::make_shared<robosense::postprocess::PostprocessNode>(cfg);
  rclcpp::spin(node);
  rclcpp::shutdown();
}
