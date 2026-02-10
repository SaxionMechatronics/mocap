#include "mocap/natnet_ros2.hpp"

// ----------------------------------------------------------------------------
// Node construction / destruction
// ----------------------------------------------------------------------------

NatNetNode::NatNetNode(const rclcpp::NodeOptions& node_options)
  : rclcpp::Node("natnet_ros2_node", node_options)
{
  // Basic connection + frame parameters (keep this minimal)
  serverIP          = this->declare_parameter<std::string>("serverIP", "192.168.0.100");
  clientIP          = this->declare_parameter<std::string>("clientIP", "192.168.0.101");
  serverType        = this->declare_parameter<std::string>("serverType", "multicast");
  multicastAddress  = this->declare_parameter<std::string>("multicastAddress", "239.255.42.99");
  serverCommandPort = this->declare_parameter<int>("serverCommandPort", 1510);
  serverDataPort    = this->declare_parameter<int>("serverDataPort", 1511);
  global_frame      = this->declare_parameter<std::string>("global_frame", "world");

  g_pClient = new NatNetClient();
  if (!connect()) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to connect to NatNet server. Shutting down node.");
    rclcpp::shutdown();
    return;
  }

  // Discover rigid bodies and create PoseStamped publishers.
  get_info();

  // Start streaming: frames will be forwarded to process_frame().
  g_pClient->SetFrameReceivedCallback(frame_callback, this);
  connected_ = true;
}

NatNetNode::~NatNetNode()
{
  if (g_pClient != nullptr) {
    g_pClient->Disconnect();
    delete g_pClient;
    g_pClient = nullptr;
  }
}

void NatNetNode::set_conn_params()
{
  // Configure the NatNet connection parameters from the current members.
  g_ConnectionType = (serverType == "unicast") ?
                      ConnectionType_Unicast :
                      ConnectionType_Multicast;

  std::memset(&g_connectParams, 0, sizeof(g_connectParams));
  g_connectParams.connectionType   = g_ConnectionType;
  g_connectParams.serverCommandPort = serverCommandPort;
  g_connectParams.serverDataPort    = serverDataPort;
  g_connectParams.serverAddress     = serverIP.c_str();
  g_connectParams.localAddress      = clientIP.c_str();
  g_connectParams.multicastAddress  =
    (serverType == "multicast") ? multicastAddress.c_str() : nullptr;
}

bool NatNetNode::connect()
{
  if (g_pClient == nullptr) {
    RCLCPP_ERROR(get_logger(), "NatNet client is null.");
    return false;
  }

  // Apply parameters into the NatNet client connect struct.
  set_conn_params();

  RCLCPP_INFO(get_logger(),
              "Connecting to NatNet server at %s (type=%s, local=%s, dataPort=%d, cmdPort=%d)...",
              serverIP.c_str(), serverType.c_str(), clientIP.c_str(),
              serverDataPort, serverCommandPort);

  const int ret = g_pClient->Connect(g_connectParams);
  if (ret != ErrorCode_OK) {
    RCLCPP_ERROR(get_logger(),
                 "Unable to connect to NatNet server. Error code: %d", ret);
    return false;
  }

  std::memset(&g_serverDescription, 0, sizeof(g_serverDescription));
  const ErrorCode desc_ret = g_pClient->GetServerDescription(&g_serverDescription);
  if (desc_ret == ErrorCode_OK && g_serverDescription.HostPresent) {
    RCLCPP_INFO(get_logger(),
                "Connected to NatNet server: %s",
                g_serverDescription.szHostComputerName);
  } else {
    RCLCPP_WARN(get_logger(),
                "NatNet server description unavailable or host not present.");
  }

  return true;
}

bool NatNetNode::disconnect()
{
  if (g_pClient == nullptr) {
    return true;
  }

  g_pClient->Disconnect();
  connected_ = false;
  return true;
}

// ----------------------------------------------------------------------------
// Data description / publishers
// ----------------------------------------------------------------------------

void NatNetNode::get_info()
{
  RCLCPP_INFO(get_logger(), "Requesting NatNet data descriptions...");

  sDataDescriptions* pDataDefs = nullptr;
  const int result = g_pClient->GetDataDescriptionList(&pDataDefs);
  if (result != ErrorCode_OK || pDataDefs == nullptr) {
    RCLCPP_ERROR(get_logger(), "Unable to retrieve NatNet data descriptions.");
    return;
  }

  RCLCPP_INFO(get_logger(),
              "Received %d data descriptions from NatNet server.",
              pDataDefs->nDataDescriptions);

  for (int i = 0; i < pDataDefs->nDataDescriptions; ++i) {
    const sDataDescription& desc = pDataDefs->arrDataDescriptions[i];
    if (desc.type == Descriptor_RigidBody) {
      sRigidBodyDescription* rb = desc.Data.RigidBodyDescription;
      if (rb == nullptr) {
        continue;
      }

      const std::string body_name(rb->szName);
      ListRigidBodies[rb->ID] = body_name;

      // Create a PoseStamped publisher for this rigid body.
      const std::string topic_name = body_name + "/pose";
      RigidbodyPub[body_name] =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, rclcpp::QoS(100));

      RCLCPP_INFO(get_logger(),
                  "Created PoseStamped publisher for rigid body '%s' (ID=%d) on topic '%s'.",
                  body_name.c_str(), rb->ID, topic_name.c_str());
    }
  }
}

void NatNetNode::del_info()
{
  ListRigidBodies.clear();
  RigidbodyPub.clear();
}

// ----------------------------------------------------------------------------
// Frame processing
// ----------------------------------------------------------------------------

std::chrono::nanoseconds NatNetNode::get_latency_info(sFrameOfMocapData* /*data*/)
{
  // In the simplified node we do not compensate for system latency.
  // Always return zero.
  return std::chrono::nanoseconds::zero();
}

void NatNetNode::process_frame(sFrameOfMocapData* data)
{
  if (data == nullptr) {
    return;
  }

  // For each rigid body in the frame, publish a PoseStamped.
  for (unsigned int i = 0; i < data->nRigidBodies; ++i) {
    process_rigid_body(data->RigidBodies[i]);
  }
}

void NatNetNode::process_rigid_body(sRigidBodyData& data)
{
  const auto it = ListRigidBodies.find(data.ID);
  if (it == ListRigidBodies.end()) {
    // Unknown rigid body ID; ignore.
    return;
  }

  const std::string& name = it->second;
  const auto pub_it = RigidbodyPub.find(name);
  if (pub_it == RigidbodyPub.end() || !pub_it->second) {
    return;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = global_frame;
  msg.header.stamp = this->now();

  msg.pose.position.x = data.x;
  msg.pose.position.y = data.y;
  msg.pose.position.z = data.z;

  msg.pose.orientation.x = data.qx;
  msg.pose.orientation.y = data.qy;
  msg.pose.orientation.z = data.qz;
  msg.pose.orientation.w = data.qw;

  pub_it->second->publish(msg);
}

// ----------------------------------------------------------------------------
// C-style callback + main
// ----------------------------------------------------------------------------

void NATNET_CALLCONV frame_callback(sFrameOfMocapData* data, void* pUserData)
{
  auto* node = static_cast<NatNetNode*>(pUserData);
  if (node != nullptr) {
    node->process_frame(data);
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<NatNetNode>(node_options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}