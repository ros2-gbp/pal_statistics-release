// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include "pal_statistics/pal_statistics_macros.hpp"

#include <map>
#include <memory>
#include <string>

namespace pal_statistics
{
typedef std::map<std::string, std::shared_ptr<StatisticsRegistry>> RegistryMap;

RegistryMap & getRegistryMap()
{
  static RegistryMap registries;
  return registries;
}

std::shared_ptr<StatisticsRegistry> getOrCreateRegistry(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock_interface,
  const std::string & topic, const std::string & key)
{
  auto & registries = getRegistryMap();
  const auto cit = registries.find(key);

  if (cit == registries.end()) {
    auto ptr =
      std::make_shared<StatisticsRegistry>(
      parameters_interface, topics_interface,
      logging_interface, clock_interface, topic);
    registries.insert(std::make_pair(key, ptr));
    return ptr;
  } else {
    return cit->second;
  }
}

std::shared_ptr<StatisticsRegistry> getRegistry(const std::string & key)
{
  const auto & registries = getRegistryMap();
  const auto cit = registries.find(key);

  if (cit == registries.end()) {
    return nullptr;
  } else {
    return cit->second;
  }
}

void deleteRegistry(const std::string & key)
{
  auto & registries = getRegistryMap();
  registries.erase(key);
}

void clearAllRegistries()
{
  auto & registries = getRegistryMap();
  registries.clear();
}

std::shared_ptr<StatisticsRegistry> getRegistry(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_interface,
  const rclcpp::node_interfaces::NodeClockInterface::SharedPtr & clock_interface,
  const std::string & /*node_namespace*/,
  const std::string & topic)
{
  return getOrCreateRegistry(
    parameters_interface, topics_interface, logging_interface, clock_interface,
    topic, topics_interface->resolve_topic_name(topic));
}
}  // namespace pal_statistics
