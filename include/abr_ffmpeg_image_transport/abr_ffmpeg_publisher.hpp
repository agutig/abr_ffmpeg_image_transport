// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef ABR_FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
#define ABR_FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_

#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <image_transport/simple_publisher_plugin.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "abr_ffmpeg_image_transport_interfaces/msg/abr_info_packet.hpp"

namespace abr_ffmpeg_image_transport
{

using ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
using FFMPEGPublisherPlugin = image_transport::SimplePublisherPlugin<FFMPEGPacket>;
using Image = sensor_msgs::msg::Image;
using FFMPEGPacketConstPtr = FFMPEGPacket::ConstSharedPtr;

/**
 * @brief Publisher plugin with ABR (Adaptive Bitrate) support on top of the ffmpeg encoder.
 */
class ABRFFMPEGPublisher : public FFMPEGPublisherPlugin
{
public:
  using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;
  using ParameterValue = rclcpp::ParameterValue;

  struct ParameterDefinition {
    ParameterValue defaultValue;
    ParameterDescriptor descriptor;
  };

  ABRFFMPEGPublisher();
  ~ABRFFMPEGPublisher() override;

  [[nodiscard]] std::string getTransportName() const noexcept override { return "abr_ffmpeg"; }

protected:
#if defined(IMAGE_TRANSPORT_API_V1) || defined(IMAGE_TRANSPORT_API_V2)
  void advertiseImpl(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos) override;
#else
  void advertiseImpl(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions opt) override;
#endif

  void publish(const Image & message, const PublishFn & publish_fn) const override;

  // Initialization state exposed to publish()
  bool init_ready = false;
  mutable bool ready_to_receive_video = false;  // renamed from ready_to_recive_video

private:
  // ---- lifecycle/helpers ----------------------------------------------------
  void packetReady(const std::string & frame_id,
                   const rclcpp::Time & stamp,
                   const std::string & codec,
                   uint32_t width,
                   uint32_t height,
                   uint64_t pts,
                   uint8_t flags,
                   uint8_t * data,
                   size_t sz);

  rmw_qos_profile_t initialize(
    rclcpp::Node * node, const std::string & base_topic, rmw_qos_profile_t custom_qos);

  void declareParameter(
    rclcpp::Node * node, const std::string & base_name, const ParameterDefinition & definition);

  // ---- logging / encoder / counters -----------------------------------------
  rclcpp::Logger logger_;
  const PublishFn * publishFunction_{nullptr};
  ffmpeg_encoder_decoder::Encoder encoder_;
  uint32_t frameCounter_{0};

  // ---- configurable parameters ----------------------------------------------
  int  performanceInterval_{175};   // frames between perf printouts
  bool measurePerformance_{false};

  // ---- ABR comms ------------------------------------------------------------
  rclcpp::Publisher<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>::SharedPtr
    abr_info_publisher_;
  rclcpp::Subscription<abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket>::SharedPtr
    abr_info_subscriber_;

  void abrInfoCallback(
    const abr_ffmpeg_image_transport_interfaces::msg::ABRInfoPacket::SharedPtr msg);

  // ---- ABR state (shared between publish() and callback) --------------------
  // NOTE: kept 'mutable' because publish() is const per image_transport API.
  mutable int   width{0};
  mutable int   height{0};
  mutable int   forced_width{0};
  mutable int   forced_height{0};
  mutable int   selected_bitrate{0};              // bps (temporary during setup)
  mutable double current_bitrate_mbps_{-1.0};     // Mbps (kept as-is per project choice)
  mutable int   selected_bitrate_init_bps_{0};    // bps (precomputed initial bitrate)

  rclcpp::Node * node_{nullptr};

  // framerate estimation / stamping
  mutable double       framerate{0.0};
  mutable rclcpp::Time framerate_ts;
  mutable rclcpp::Time last_update_time{rclcpp::Time(0, 0, RCL_STEADY_TIME)};

  // bitrate ladder & serialization
  mutable std::vector<double> bitrate_ladder;     // Mbps
  mutable std::string         bitrate_ladder_string; // serialized JSON of ladder
  mutable bool                ladder_ready{false};

  // resolution mapping and app configuration
  std::unordered_map<std::string, std::pair<int, int>> resolution_map_;
  nlohmann::json app_config_json_;

  // ---- utilities -------------------------------------------------------------
  std::string identifyResolution(int width, int height) const;
  std::pair<int, int> parseResolution(const std::string & res_str) const;
  void expectedBitrateLadder(int width, int height, int framerate) const;
  void filterBitrateLadder(double max_rate_mbps) const;
};

}  // namespace abr_ffmpeg_image_transport

#endif  // ABR_FFMPEG_IMAGE_TRANSPORT__FFMPEG_PUBLISHER_HPP_
