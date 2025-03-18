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

 #ifndef POSTPROCESS_YAML_READER_H_
 #define POSTPROCESS_YAML_READER_H_

#include <yaml-cpp/yaml.h>

namespace robosense {
namespace postprocess {

template <typename T>
inline void yamlReadAbort(const YAML::Node& yaml, const std::string& key, T& out_val) {
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null) {
    exit(-1);
  }
  else {
    out_val = yaml[key].as<T>();
  }
}

template <typename T>
inline bool yamlRead(const YAML::Node& yaml, const std::string& key, T& out_val, const T& default_val) {
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null) {
    out_val = default_val;
    return false;
  } else {
    out_val = yaml[key].as<T>();
    return true;
  }
}

inline YAML::Node yamlSubNodeAbort(const YAML::Node& yaml, const std::string& node) {
  YAML::Node ret = yaml[node.c_str()];
  if (!ret) {
    exit(-1);
  }
  return ret;
}

}  // namespace postprocess
}  // namespace robosense
#endif // POSTPROCESS_YAML_READER_H_
