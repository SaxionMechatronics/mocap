#pragma once

#include <map>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <NatNetCAPI.h>
#include <NatNetClient.h>

class NatNetNode : public rclcpp::Node
{
public:
  explicit NatNetNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  ~NatNetNode();

  // Connection / config
  bool disconnect();
  bool connect();
  void set_conn_params();

  // NatNet related
  void process_frame(sFrameOfMocapData* data);

  void get_info();
  void del_info();
  std::chrono::nanoseconds get_latency_info(sFrameOfMocapData* data);

  void process_rigid_body(sRigidBodyData& data);

private:
  // Helpers
  void setup_parameters_();
  void setup_publishers_();
  void setup_tf_();
  void maybe_connect_();   // optional: connect on startup if desired

  // NatNet internals
  NatNetClient* g_pClient{nullptr};
  sNatNetClientConnectParams g_connectParams{};
  ConnectionType g_ConnectionType{ConnectionType_Multicast};
  sServerDescription g_serverDescription{};

  std::string serverIP;
  std::string clientIP;
  std::string serverType;
  std::string multicastAddress;
  int serverCommandPort{0};
  int serverDataPort{0};
  std::string global_frame;

  // ROS publishers
  std::map<int32_t, std::string> ListRigidBodies;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> RigidbodyPub;

  // Optional: keep a simple “connected” state
  bool connected_{false};
};

void NATNET_CALLCONV frame_callback(sFrameOfMocapData* data, void* pUserData);
