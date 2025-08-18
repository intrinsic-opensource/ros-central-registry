// Copyright 2023 Sony Group Corporation.
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

#include "rmw_dds_common/context.hpp"

#include <mutex>
#include <string>

#include "rmw/types.h"

#include "rmw_dds_common/msg/participant_entities_info.hpp"

namespace rmw_dds_common
{

static bool call_publish_callback(
  const rmw_publisher_t * pub,
  const Context::publish_callback_t & publish_callback,
  const rmw_dds_common::msg::ParticipantEntitiesInfo & msg)
{
  if (nullptr == pub || nullptr == publish_callback ||
    RMW_RET_OK != publish_callback(pub, static_cast<const void *>(&msg)))
  {
    return false;
  }
  return true;
}

rmw_ret_t Context::add_node_graph(
  const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.add_node(gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    graph_cache.remove_node(gid, name, namespace_);
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::remove_node_graph(
  const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.remove_node(gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::add_subscriber_graph(
  const rmw_gid_t & subscription_gid, const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.associate_reader(
    subscription_gid, gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    static_cast<void>(graph_cache.dissociate_reader(
      subscription_gid, gid, name, namespace_));
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::remove_subscriber_graph(
  const rmw_gid_t & subscription_gid, const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.dissociate_reader(
    subscription_gid, gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::add_publisher_graph(
  const rmw_gid_t & publisher_gid, const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.associate_writer(
    publisher_gid, gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    static_cast<void>(graph_cache.dissociate_writer(
      publisher_gid, gid, name, namespace_));
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::remove_publisher_graph(
  const rmw_gid_t & publisher_gid, const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.dissociate_writer(
    publisher_gid, gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::add_client_graph(
  const rmw_gid_t & request_publisher_gid, const rmw_gid_t & response_subscriber_gid,
  const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  graph_cache.associate_writer(
    request_publisher_gid, gid, name, namespace_);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.associate_reader(
    response_subscriber_gid, gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    static_cast<void>(graph_cache.dissociate_reader(
      response_subscriber_gid, gid, name, namespace_));
    static_cast<void>(graph_cache.dissociate_writer(
      request_publisher_gid, gid, name, namespace_));
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::remove_client_graph(
  const rmw_gid_t & request_publisher_gid, const rmw_gid_t & response_subscriber_gid,
  const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  graph_cache.dissociate_writer(
    request_publisher_gid, gid, name, namespace_);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.dissociate_reader(
    response_subscriber_gid, gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::add_service_graph(
  const rmw_gid_t & request_subscriber_gid, const rmw_gid_t & response_publisher_gid,
  const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  graph_cache.associate_reader(
    request_subscriber_gid, gid, name, namespace_);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.associate_writer(
    response_publisher_gid, gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    static_cast<void>(graph_cache.dissociate_writer(
      response_publisher_gid, gid, name, namespace_));
    static_cast<void>(graph_cache.dissociate_reader(
      request_subscriber_gid, gid, name, namespace_));
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::remove_service_graph(
  const rmw_gid_t & request_subscriber_gid, const rmw_gid_t & response_publisher_gid,
  const std::string & name, const std::string & namespace_)
{
  std::lock_guard<std::mutex> guard(node_update_mutex);
  graph_cache.dissociate_reader(
    request_subscriber_gid, gid, name, namespace_);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.dissociate_writer(
    response_publisher_gid, gid, name, namespace_);

  if (!call_publish_callback(pub, publish_callback, msg)) {
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

}  // namespace rmw_dds_common
