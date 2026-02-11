#pragma once

#include <map>
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

  // NatNet related
  void process_frame(sFrameOfMocapData* data);

  void get_info();
  void del_info();

  void process_rigid_body(sRigidBodyData& data);

private:
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
};

void NATNET_CALLCONV frame_callback(sFrameOfMocapData* data, void* pUserData);
