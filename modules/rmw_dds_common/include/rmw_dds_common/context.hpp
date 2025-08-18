// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_DDS_COMMON__CONTEXT_HPP_
#define RMW_DDS_COMMON__CONTEXT_HPP_

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "rmw/types.h"

#include "rmw_dds_common/graph_cache.hpp"
#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common
{

/// Base data structure that a Context will need in any DDS-based RMW implementation
/// mapping one Participant to Multiple Nodes.
struct Context
{
  /// Global ID of the Participant that the Context uses.
  rmw_gid_t gid;
  /// Publisher used to publish ParticipantEntitiesInfo discovery data.
  rmw_publisher_t * pub;
  /// Subscriber used to listen to ParticipantEntitiesInfo discovery data.
  rmw_subscription_t * sub;
  /// Cached graph from discovery data.
  GraphCache graph_cache;
  /// Thread to listen to discovery data.
  std::thread listener_thread;
  /// Indicates if the listener thread is running.
  std::atomic_bool thread_is_running;
  /// Awakes listener thread when finishing the context.
  rmw_guard_condition_t * listener_thread_gc;
  /// Guard condition that should be triggered when the graph changes.
  rmw_guard_condition_t * graph_guard_condition;

  using publish_callback_t =
    std::function<rmw_ret_t(const rmw_publisher_t * pub, const void * msg)>;
  /// Publish a graph message when updating or destroying graph cache.
  publish_callback_t publish_callback;

  /// Add graph for creating a node.
  /**
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  add_node_graph(
    const std::string & name, const std::string & namespace_);

  /// Remove graph for destroying a node.
  /**
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  remove_node_graph(
    const std::string & name, const std::string & namespace_);

  /// Add graph for creating a subscription.
  /**
   * \param subscription_gid subscription gid.
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  add_subscriber_graph(
    const rmw_gid_t & subscription_gid, const std::string & name, const std::string & namespace_);

  /// Remove graph for destroying a subscription.
  /**
   * \param subscription_gid subscription gid.
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  remove_subscriber_graph(
    const rmw_gid_t & subscription_gid, const std::string & name, const std::string & namespace_);

  /// Add graph for creating a publisher.
  /**
   * \param publisher_gid publisher gid.
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  add_publisher_graph(
    const rmw_gid_t & publisher_gid, const std::string & name, const std::string & namespace_);

  /// Remove graph for destroying a publisher.
  /**
   * \param publisher_gid publisher gid.
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  remove_publisher_graph(
    const rmw_gid_t & publisher_gid, const std::string & name, const std::string & namespace_);

  /// Add graph for creating a client.
  /**
   * \param request_publisher_gid request publisher gid of the client.
   * \param response_subscriber_gid response subscriber gid of the client.
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  add_client_graph(
    const rmw_gid_t & request_publisher_gid, const rmw_gid_t & response_subscriber_gid,
    const std::string & name, const std::string & namespace_);

  /// Remove graph for destroying a client.
  /**
   * \param request_publisher_gid request publisher gid of the client.
   * \param response_subscriber_gid response subscriber gid of the client.
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  remove_client_graph(
    const rmw_gid_t & request_publisher_gid, const rmw_gid_t & response_subscriber_gid,
    const std::string & name, const std::string & namespace_);

  /// Add graph for creating a service.
  /**
   * \param request_subscriber_gid request subscriber gid of the client.
   * \param response_publisher_gid response publisher gid of the client.
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  add_service_graph(
    const rmw_gid_t & request_subscriber_gid, const rmw_gid_t & response_publisher_gid,
    const std::string & name, const std::string & namespace_);

  /// Remove graph for destroying a service.
  /**
   * \param request_subscriber_gid request subscriber gid of the client.
   * \param response_publisher_gid response publisher gid of the client.
   * \param name node name.
   * \param namespace_ node namespace.
   * \return `RMW_RET_OK` if successful, or
   * \return `RMW_RET_ERROR` an unexpected error occurs.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  remove_service_graph(
    const rmw_gid_t & request_subscriber_gid, const rmw_gid_t & response_publisher_gid,
    const std::string & name, const std::string & namespace_);

private:
  /// Mutex that should be locked when updating graph cache and publishing a graph message.
  /// Though graph_cache methods are thread safe, both cache update and publishing have to also
  /// be atomic.
  std::mutex node_update_mutex;
};

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__CONTEXT_HPP_
