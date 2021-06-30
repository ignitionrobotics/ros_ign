// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <exception>
#include <ros/console.h>

#include "ros_ign_bridge/convert.hpp"

namespace ros_ign_bridge
{

// This can be used to replace `::` with `/` to make frame_id compatible with TF
std::string replace_delimiter(const std::string &input,
                              const std::string &old_delim,
                              const std::string new_delim)
{
  std::string output;
  output.reserve(input.size());

  std::size_t last_pos = 0;

  while (last_pos < input.size())
  {
    std::size_t pos = input.find(old_delim, last_pos);
    output += input.substr(last_pos, pos - last_pos);
    if (pos != std::string::npos)
    {
      output += new_delim;
      pos += old_delim.size();
    }

    last_pos = pos;
  }

  return output;
}

// Frame id from ROS to ign is not supported right now
// std::string frame_id_ros_to_ign(const std::string &frame_id)
// {
//   return replace_delimiter(frame_id, "/", "::");
// }

std::string frame_id_ign_to_ros(const std::string &frame_id)
{
  //return replace_delimiter(frame_id, "::", "/");
  //ROS sensor header frame_id should be unique and without prefix
  return frame_id.substr(frame_id.find_last_of("::") + 1);
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Bool & ros_msg,
  ignition::msgs::Boolean & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Boolean & ign_msg,
  std_msgs::Bool & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Empty &,
  ignition::msgs::Empty &)
{
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Empty &,
  std_msgs::Empty &)
{
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Int32 & ros_msg,
  ignition::msgs::Int32 & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Int32 & ign_msg,
  std_msgs::Int32 & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Float32 & ros_msg,
  ignition::msgs::Float & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Float & ign_msg,
  std_msgs::Float32 & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Float64 & ros_msg,
  ignition::msgs::Double & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Double & ign_msg,
  std_msgs::Float64 & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ros_to_ign(
  const std_msgs::Header & ros_msg,
  ignition::msgs::Header & ign_msg)
{
  ign_msg.mutable_stamp()->set_sec(ros_msg.stamp.sec);
  ign_msg.mutable_stamp()->set_nsec(ros_msg.stamp.nsec);
  auto newPair = ign_msg.add_data();
  newPair->set_key("seq");
  newPair->add_value(std::to_string(ros_msg.seq));
  newPair = ign_msg.add_data();
  newPair->set_key("frame_id");
  newPair->add_value(ros_msg.frame_id);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Header & ign_msg,
  std_msgs::Header & ros_msg)
{
  ros_msg.stamp = ros::Time(ign_msg.stamp().sec(), ign_msg.stamp().nsec());
  for (auto i = 0; i < ign_msg.data_size(); ++i)
  {
    auto aPair = ign_msg.data(i);
    if (aPair.key() == "seq" && aPair.value_size() > 0)
    {
      std::string value = aPair.value(0);
      try
      {
        unsigned long ul = std::stoul(value, nullptr);
        ros_msg.seq = ul;
      }
      catch (std::exception & e)
      {
        ROS_ERROR_STREAM("Exception converting [" << value << "] to an "
                  << "unsigned int" << std::endl);
      }
    }
    else if (aPair.key() == "frame_id" && aPair.value_size() > 0)
    {
      ros_msg.frame_id = frame_id_ign_to_ros(aPair.value(0));
    }
  }
}

template<>
void
convert_ros_to_ign(
  const std_msgs::String & ros_msg,
  ignition::msgs::StringMsg & ign_msg)
{
  ign_msg.set_data(ros_msg.data);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::StringMsg & ign_msg,
  std_msgs::String & ros_msg)
{
  ros_msg.data = ign_msg.data();
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Clock & ign_msg,
  rosgraph_msgs::Clock & ros_msg)
{
  ros_msg.clock = ros::Time(ign_msg.sim().sec(), ign_msg.sim().nsec());
}

template<>
void
convert_ros_to_ign(
  const rosgraph_msgs::Clock & ros_msg,
  ignition::msgs::Clock & ign_msg)
{
  ign_msg.mutable_sim()->set_sec(ros_msg.clock.sec);
  ign_msg.mutable_sim()->set_nsec(ros_msg.clock.nsec);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Quaternion & ros_msg,
  ignition::msgs::Quaternion & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
  ign_msg.set_w(ros_msg.w);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Quaternion & ign_msg,
  geometry_msgs::Quaternion & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
  ros_msg.w = ign_msg.w();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Vector3 & ros_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Vector3 & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Point & ros_msg,
  ignition::msgs::Vector3d & ign_msg)
{
  ign_msg.set_x(ros_msg.x);
  ign_msg.set_y(ros_msg.y);
  ign_msg.set_z(ros_msg.z);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Vector3d & ign_msg,
  geometry_msgs::Point & ros_msg)
{
  ros_msg.x = ign_msg.x();
  ros_msg.y = ign_msg.y();
  ros_msg.z = ign_msg.z();
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Pose & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.position, *ign_msg.mutable_position());
  convert_ros_to_ign(ros_msg.orientation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Pose & ros_msg)
{
  convert_ign_to_ros(ign_msg.position(), ros_msg.position);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.orientation);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::PoseStamped & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.pose, ign_msg);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::PoseStamped & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg, ros_msg.pose);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Transform & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.translation , *ign_msg.mutable_position());
  convert_ros_to_ign(ros_msg.rotation, *ign_msg.mutable_orientation());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::Transform & ros_msg)
{
  convert_ign_to_ros(ign_msg.position(), ros_msg.translation);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.rotation);
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::TransformStamped & ros_msg,
  ignition::msgs::Pose & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.transform, ign_msg);

  auto newPair = ign_msg.mutable_header()->add_data();
  newPair->set_key("child_frame_id");
  newPair->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose & ign_msg,
  geometry_msgs::TransformStamped & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg, ros_msg.transform);
  for (auto i = 0; i < ign_msg.header().data_size(); ++i)
  {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros_msg.child_frame_id = frame_id_ign_to_ros(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_ros_to_ign(
  const tf2_msgs::TFMessage & ros_msg,
  ignition::msgs::Pose_V & ign_msg)
{
  ign_msg.clear_pose();
  for (auto const &t : ros_msg.transforms)
  {
    auto p = ign_msg.add_pose();
    convert_ros_to_ign(t, *p);
  }

  if (!ros_msg.transforms.empty())
  {
    convert_ros_to_ign(ros_msg.transforms[0].header,
        (*ign_msg.mutable_header()));
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Pose_V & ign_msg,
  tf2_msgs::TFMessage & ros_msg)
{
  ros_msg.transforms.clear();
  for (auto const &p : ign_msg.pose())
  {
    geometry_msgs::TransformStamped tf;
    convert_ign_to_ros(p, tf);
    ros_msg.transforms.push_back(tf);
  }
}

template<>
void
convert_ros_to_ign(
  const geometry_msgs::Twist & ros_msg,
  ignition::msgs::Twist & ign_msg)
{
  convert_ros_to_ign(ros_msg.linear,  (*ign_msg.mutable_linear()));
  convert_ros_to_ign(ros_msg.angular, (*ign_msg.mutable_angular()));
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Twist & ign_msg,
  geometry_msgs::Twist & ros_msg)
{
  convert_ign_to_ros(ign_msg.linear(), ros_msg.linear);
  convert_ign_to_ros(ign_msg.angular(), ros_msg.angular);
}

// template<>
// void
// convert_ros_to_ign(
//   const mav_msgs::Actuators & ros_msg,
//   ignition::msgs::Actuators & ign_msg)
// {
//   convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
//
//   for (auto i = 0u; i < ros_msg.angles.size(); ++i)
//     ign_msg.add_position(ros_msg.angles[i]);
//   for (auto i = 0u; i < ros_msg.angular_velocities.size(); ++i)
//     ign_msg.add_velocity(ros_msg.angular_velocities[i]);
//   for (auto i = 0u; i < ros_msg.normalized.size(); ++i)
//     ign_msg.add_normalized(ros_msg.normalized[i]);
// }
//
// template<>
// void
// convert_ign_to_ros(
//   const ignition::msgs::Actuators & ign_msg,
//   mav_msgs::Actuators & ros_msg)
// {
//   convert_ign_to_ros(ign_msg.header(), ros_msg.header);
//
//   for (auto i = 0; i < ign_msg.position_size(); ++i)
//     ros_msg.angles.push_back(ign_msg.position(i));
//   for (auto i = 0; i < ign_msg.velocity_size(); ++i)
//     ros_msg.angular_velocities.push_back(ign_msg.velocity(i));
//   for (auto i = 0; i < ign_msg.normalized_size(); ++i)
//     ros_msg.normalized.push_back(ign_msg.normalized(i));
// }

template<>
void
convert_ros_to_ign(
  const nav_msgs::Odometry & ros_msg,
  ignition::msgs::Odometry & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.pose.pose, (*ign_msg.mutable_pose()));
  convert_ros_to_ign(ros_msg.twist.twist, (*ign_msg.mutable_twist()));

  auto childFrame = ign_msg.mutable_header()->add_data();
  childFrame->set_key("child_frame_id");
  childFrame->add_value(ros_msg.child_frame_id);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Odometry & ign_msg,
  nav_msgs::Odometry & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg.pose(), ros_msg.pose.pose);
  convert_ign_to_ros(ign_msg.twist(), ros_msg.twist.twist);

  for (auto i = 0; i < ign_msg.header().data_size(); ++i)
  {
    auto aPair = ign_msg.header().data(i);
    if (aPair.key() == "child_frame_id" && aPair.value_size() > 0)
    {
      ros_msg.child_frame_id = frame_id_ign_to_ros(aPair.value(0));
      break;
    }
  }
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::FluidPressure & ros_msg,
  ignition::msgs::FluidPressure & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  ign_msg.set_pressure(ros_msg.fluid_pressure);
  ign_msg.set_variance(ros_msg.variance);
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::FluidPressure & ign_msg,
  sensor_msgs::FluidPressure & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  ros_msg.fluid_pressure = ign_msg.pressure();
  ros_msg.variance = ign_msg.variance();
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::Image & ros_msg,
  ignition::msgs::Image & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  ign_msg.set_width(ros_msg.width);
  ign_msg.set_height(ros_msg.height);

  unsigned int num_channels;
  unsigned int octets_per_channel;

  if (ros_msg.encoding == "mono8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::L_INT8);
    num_channels = 1;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "mono16")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::L_INT16);
    num_channels = 1;
    octets_per_channel = 2u;
  }
  else if (ros_msg.encoding == "rgb8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::RGB_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "rgba8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::RGBA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "bgra8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::BGRA_INT8);
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "rgb16")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::RGB_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ros_msg.encoding == "bgr8")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::BGR_INT8);
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ros_msg.encoding == "bgr16")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::BGR_INT16);
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ros_msg.encoding == "32FC1")
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::R_FLOAT32);
    num_channels = 1;
    octets_per_channel = 4u;
  }
  else
  {
    ign_msg.set_pixel_format_type(
      ignition::msgs::PixelFormatType::UNKNOWN_PIXEL_FORMAT);
    ROS_ERROR_STREAM("Unsupported pixel format [" << ros_msg.encoding << "]"
              << std::endl);
    return;
  }

  ign_msg.set_step(ign_msg.width() * num_channels * octets_per_channel);

  ign_msg.set_data(&(ros_msg.data[0]), ign_msg.step() * ign_msg.height());
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Image & ign_msg,
  sensor_msgs::Image & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  ros_msg.height = ign_msg.height();
  ros_msg.width = ign_msg.width();

  unsigned int num_channels;
  unsigned int octets_per_channel;

  if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::L_INT8)
  {
    ros_msg.encoding = "mono8";
    num_channels = 1;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::L_INT16)
  {
    ros_msg.encoding = "mono16";
    num_channels = 1;
    octets_per_channel = 2u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::RGB_INT8)
  {
    ros_msg.encoding = "rgb8";
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::RGBA_INT8)
  {
    ros_msg.encoding = "rgba8";
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::BGRA_INT8)
  {
    ros_msg.encoding = "bgra8";
    num_channels = 4;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::RGB_INT16)
  {
    ros_msg.encoding = "rgb16";
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::BGR_INT8)
  {
    ros_msg.encoding = "bgr8";
    num_channels = 3;
    octets_per_channel = 1u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::BGR_INT16)
  {
    ros_msg.encoding = "bgr16";
    num_channels = 3;
    octets_per_channel = 2u;
  }
  else if (ign_msg.pixel_format_type() ==
      ignition::msgs::PixelFormatType::R_FLOAT32)
  {
    ros_msg.encoding = "32FC1";
    num_channels = 1;
    octets_per_channel = 4u;
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported pixel format ["
        << ign_msg.pixel_format_type() << "]" << std::endl);
    return;
  }

  ros_msg.is_bigendian = false;
  ros_msg.step = ros_msg.width * num_channels * octets_per_channel;

  auto count = ros_msg.step * ros_msg.height;
  ros_msg.data.resize(ros_msg.step * ros_msg.height);
  std::copy(
    ign_msg.data().begin(),
    ign_msg.data().begin() + count,
    ros_msg.data.begin());
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::CameraInfo & ros_msg,
  ignition::msgs::CameraInfo & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  ign_msg.set_width(ros_msg.width);
  ign_msg.set_height(ros_msg.height);

  auto distortion = ign_msg.mutable_distortion();
  if (ros_msg.distortion_model == "plumb_bob")
  {
    distortion->set_model(ignition::msgs::CameraInfo::Distortion::PLUMB_BOB);
  }
  else if (ros_msg.distortion_model == "rational_polynomial")
  {
    distortion->set_model(ignition::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL);
  }
  else if (ros_msg.distortion_model == "equidistant")
  {
    distortion->set_model(ignition::msgs::CameraInfo::Distortion::EQUIDISTANT);
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported distortion model ["
        << ros_msg.distortion_model << "]" << std::endl);
  }
  for (auto i = 0u; i < ros_msg.D.size(); ++i)
  {
    distortion->add_k(ros_msg.D[i]);
  }

  auto intrinsics = ign_msg.mutable_intrinsics();
  for (auto i = 0u; i < ros_msg.K.size(); ++i)
  {
    intrinsics->add_k(ros_msg.K[i]);
  }

  auto projection = ign_msg.mutable_projection();
  for (auto i = 0u; i < ros_msg.P.size(); ++i)
  {
    projection->add_p(ros_msg.P[i]);
  }

  for (auto i = 0u; i < ros_msg.R.size(); ++i)
  {
    ign_msg.add_rectification_matrix(ros_msg.R[i]);
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::CameraInfo & ign_msg,
  sensor_msgs::CameraInfo & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  ros_msg.height = ign_msg.height();
  ros_msg.width = ign_msg.width();

  auto distortion = ign_msg.distortion();
  if (ign_msg.has_distortion())
  {
    auto distortion = ign_msg.distortion();
    if (distortion.model() ==
        ignition::msgs::CameraInfo::Distortion::PLUMB_BOB)
    {
      ros_msg.distortion_model = "plumb_bob";
    }
    else if (distortion.model() ==
        ignition::msgs::CameraInfo::Distortion::RATIONAL_POLYNOMIAL)
    {
      ros_msg.distortion_model = "rational_polynomial";
    }
    else if (distortion.model() ==
        ignition::msgs::CameraInfo::Distortion::EQUIDISTANT)
    {
      ros_msg.distortion_model = "equidistant";
    }
    else
    {
      ROS_ERROR_STREAM("Unsupported distortion model ["
                << distortion.model() << "]" << std::endl);
    }

    ros_msg.D.resize(distortion.k_size());
    for (auto i = 0; i < distortion.k_size(); ++i)
    {
      ros_msg.D[i] = distortion.k(i);
    }
  }

  auto intrinsics = ign_msg.intrinsics();
  if (ign_msg.has_intrinsics())
  {
    auto intrinsics = ign_msg.intrinsics();

    for (auto i = 0; i < intrinsics.k_size(); ++i)
    {
      ros_msg.K[i] = intrinsics.k(i);
    }
  }

  auto projection = ign_msg.projection();
  if (ign_msg.has_projection())
  {
    auto projection = ign_msg.projection();

    for (auto i = 0; i < projection.p_size(); ++i)
    {
      ros_msg.P[i] = projection.p(i);
    }
  }

  for (auto i = 0; i < ign_msg.rectification_matrix_size(); ++i)
  {
    ros_msg.R[i] = ign_msg.rectification_matrix(i);
  }
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::Imu & ros_msg,
  ignition::msgs::IMU & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  // ToDo: Verify that this is the expected value (probably not).
  ign_msg.set_entity_name(ros_msg.header.frame_id);

  convert_ros_to_ign(ros_msg.orientation, (*ign_msg.mutable_orientation()));
  convert_ros_to_ign(ros_msg.angular_velocity,
                   (*ign_msg.mutable_angular_velocity()));
  convert_ros_to_ign(ros_msg.linear_acceleration,
                   (*ign_msg.mutable_linear_acceleration()));
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::IMU & ign_msg,
  sensor_msgs::Imu & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg.orientation(), ros_msg.orientation);
  convert_ign_to_ros(ign_msg.angular_velocity(), ros_msg.angular_velocity);
  convert_ign_to_ros(ign_msg.linear_acceleration(), ros_msg.linear_acceleration);

  // Covariances not supported in Ignition::msgs::IMU
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::JointState & ros_msg,
  ignition::msgs::Model & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  for (auto i = 0u; i < ros_msg.position.size(); ++i)
  {
    auto newJoint = ign_msg.add_joint();
    newJoint->set_name(ros_msg.name[i]);
    newJoint->mutable_axis1()->set_position(ros_msg.position[i]);
    newJoint->mutable_axis1()->set_velocity(ros_msg.velocity[i]);
    newJoint->mutable_axis1()->set_force(ros_msg.effort[i]);
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Model & ign_msg,
  sensor_msgs::JointState & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  for (auto i = 0; i < ign_msg.joint_size(); ++i)
  {
    ros_msg.name.push_back(ign_msg.joint(i).name());
    ros_msg.position.push_back(ign_msg.joint(i).axis1().position());
    ros_msg.velocity.push_back(ign_msg.joint(i).axis1().velocity());
    ros_msg.effort.push_back(ign_msg.joint(i).axis1().force());
  }
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::LaserScan & ros_msg,
  ignition::msgs::LaserScan & ign_msg)
{
  const unsigned int num_readings =
    (ros_msg.angle_max - ros_msg.angle_min) / ros_msg.angle_increment;

  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  ign_msg.set_frame(ros_msg.header.frame_id);
  ign_msg.set_angle_min(ros_msg.angle_min);
  ign_msg.set_angle_max(ros_msg.angle_max);
  ign_msg.set_angle_step(ros_msg.angle_increment);
  ign_msg.set_range_min(ros_msg.range_min);
  ign_msg.set_range_max(ros_msg.range_max);
  ign_msg.set_count(num_readings);

  // Not supported in sensor_msgs::LaserScan.
  ign_msg.set_vertical_angle_min(0.0);
  ign_msg.set_vertical_angle_max(0.0);
  ign_msg.set_vertical_angle_step(0.0);
  ign_msg.set_vertical_count(0u);

  for (auto i = 0u; i < ign_msg.count(); ++i)
  {
    ign_msg.add_ranges(ros_msg.ranges[i]);
    ign_msg.add_intensities(ros_msg.intensities[i]);
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::LaserScan & ign_msg,
  sensor_msgs::LaserScan & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  ros_msg.header.frame_id = frame_id_ign_to_ros(ign_msg.frame());

  ros_msg.angle_min = ign_msg.angle_min();
  ros_msg.angle_max = ign_msg.angle_max();
  ros_msg.angle_increment = ign_msg.angle_step();

  // Not supported in ignition::msgs::LaserScan.
  ros_msg.time_increment = 0.0;
  ros_msg.scan_time = 0.0;

  ros_msg.range_min = ign_msg.range_min();
  ros_msg.range_max = ign_msg.range_max();

  auto count = ign_msg.count();
  auto vertical_count = ign_msg.vertical_count();

  // If there are multiple vertical beams, use the one in the middle.
  size_t start = (vertical_count / 2) * count;

  // Copy ranges into ROS message.
  ros_msg.ranges.resize(count);
  std::copy(
    ign_msg.ranges().begin() + start,
    ign_msg.ranges().begin() + start + count,
    ros_msg.ranges.begin());

  // Copy intensities into ROS message.
  ros_msg.intensities.resize(count);
  std::copy(
    ign_msg.intensities().begin() + start,
    ign_msg.intensities().begin() + start + count,
    ros_msg.intensities.begin());
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::MagneticField & ros_msg,
  ignition::msgs::Magnetometer & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));
  convert_ros_to_ign(ros_msg.magnetic_field, (*ign_msg.mutable_field_tesla()));
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::Magnetometer & ign_msg,
  sensor_msgs::MagneticField & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);
  convert_ign_to_ros(ign_msg.field_tesla(), ros_msg.magnetic_field);

  // magnetic_field_covariance is not supported in Ignition::Msgs::Magnetometer.
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::PointCloud2 & ros_msg,
  ignition::msgs::PointCloudPacked &ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  ign_msg.set_height(ros_msg.height);
  ign_msg.set_width(ros_msg.width);
  ign_msg.set_is_bigendian(ros_msg.is_bigendian);
  ign_msg.set_point_step(ros_msg.point_step);
  ign_msg.set_row_step(ros_msg.row_step);
  ign_msg.set_is_dense(ros_msg.is_dense);
  ign_msg.mutable_data()->resize(ros_msg.data.size());
  memcpy(ign_msg.mutable_data()->data(), ros_msg.data.data(),
         ros_msg.data.size());

  for (unsigned int i = 0; i < ros_msg.fields.size(); ++i)
  {
    ignition::msgs::PointCloudPacked::Field *pf = ign_msg.add_field();
    pf->set_name(ros_msg.fields[i].name);
    pf->set_count(ros_msg.fields[i].count);
    pf->set_offset(ros_msg.fields[i].offset);
    switch (ros_msg.fields[i].datatype)
    {
      default:
      case sensor_msgs::PointField::INT8:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::INT8);
        break;
      case sensor_msgs::PointField::UINT8:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::UINT8);
        break;
      case sensor_msgs::PointField::INT16:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::INT16);
        break;
      case sensor_msgs::PointField::UINT16:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::UINT16);
        break;
      case sensor_msgs::PointField::INT32:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::INT32);
        break;
      case sensor_msgs::PointField::UINT32:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::UINT32);
        break;
      case sensor_msgs::PointField::FLOAT32:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::FLOAT32);
        break;
      case sensor_msgs::PointField::FLOAT64:
        pf->set_datatype(ignition::msgs::PointCloudPacked::Field::FLOAT64);
        break;
    };
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::PointCloudPacked & ign_msg,
  sensor_msgs::PointCloud2 & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  ros_msg.height = ign_msg.height();
  ros_msg.width = ign_msg.width();
  ros_msg.is_bigendian = ign_msg.is_bigendian();
  ros_msg.point_step = ign_msg.point_step();
  ros_msg.row_step = ign_msg.row_step();
  ros_msg.is_dense = ign_msg.is_dense();
  ros_msg.data.resize(ign_msg.data().size());
  memcpy(ros_msg.data.data(), ign_msg.data().c_str(), ign_msg.data().size());

  for (int i = 0; i < ign_msg.field_size(); ++i)
  {
    sensor_msgs::PointField pf;
    pf.name = ign_msg.field(i).name();
    pf.count = ign_msg.field(i).count();
    pf.offset = ign_msg.field(i).offset();
    switch (ign_msg.field(i).datatype())
    {
      default:
      case ignition::msgs::PointCloudPacked::Field::INT8:
        pf.datatype = sensor_msgs::PointField::INT8;
        break;
      case ignition::msgs::PointCloudPacked::Field::UINT8:
        pf.datatype = sensor_msgs::PointField::UINT8;
        break;
      case ignition::msgs::PointCloudPacked::Field::INT16:
        pf.datatype = sensor_msgs::PointField::INT16;
        break;
      case ignition::msgs::PointCloudPacked::Field::UINT16:
        pf.datatype = sensor_msgs::PointField::UINT16;
        break;
      case ignition::msgs::PointCloudPacked::Field::INT32:
        pf.datatype = sensor_msgs::PointField::INT32;
        break;
      case ignition::msgs::PointCloudPacked::Field::UINT32:
        pf.datatype = sensor_msgs::PointField::UINT32;
        break;
      case ignition::msgs::PointCloudPacked::Field::FLOAT32:
        pf.datatype = sensor_msgs::PointField::FLOAT32;
        break;
      case ignition::msgs::PointCloudPacked::Field::FLOAT64:
        pf.datatype = sensor_msgs::PointField::FLOAT64;
        break;
    };
    ros_msg.fields.push_back(pf);
  }
}

template<>
void
convert_ros_to_ign(
  const sensor_msgs::BatteryState & ros_msg,
  ignition::msgs::BatteryState & ign_msg)
{
  convert_ros_to_ign(ros_msg.header, (*ign_msg.mutable_header()));

  ign_msg.set_voltage(ros_msg.voltage);
  ign_msg.set_current(ros_msg.current);
  ign_msg.set_charge(ros_msg.charge);
  ign_msg.set_capacity(ros_msg.capacity);
  ign_msg.set_percentage(ros_msg.percentage);

  if (ros_msg.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN)
  {
    ign_msg.set_power_supply_status(ignition::msgs::BatteryState::UNKNOWN);
  }
  else if (ros_msg.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING)
  {
    ign_msg.set_power_supply_status(ignition::msgs::BatteryState::CHARGING);
  }
  else if (ros_msg.power_supply_status ==
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING)
  {
    ign_msg.set_power_supply_status(ignition::msgs::BatteryState::DISCHARGING);
  }
  else if (ros_msg.power_supply_status ==
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING)
  {
    ign_msg.set_power_supply_status(ignition::msgs::BatteryState::NOT_CHARGING);
  }
  else if (ros_msg.power_supply_status == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL)
  {
    ign_msg.set_power_supply_status(ignition::msgs::BatteryState::FULL);
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported power supply status ["
        << ros_msg.power_supply_status << "]" << std::endl);
  }
}

template<>
void
convert_ign_to_ros(
  const ignition::msgs::BatteryState & ign_msg,
  sensor_msgs::BatteryState & ros_msg)
{
  convert_ign_to_ros(ign_msg.header(), ros_msg.header);

  ros_msg.voltage = ign_msg.voltage();
  ros_msg.current = ign_msg.current();
  ros_msg.charge = ign_msg.charge();
  ros_msg.capacity = ign_msg.capacity();
  ros_msg.design_capacity = std::numeric_limits<double>::quiet_NaN();
  ros_msg.percentage = ign_msg.percentage();

  if (ign_msg.power_supply_status() ==
      ignition::msgs::BatteryState::UNKNOWN)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }
  else if (ign_msg.power_supply_status() ==
      ignition::msgs::BatteryState::CHARGING)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  }
  else if (ign_msg.power_supply_status() ==
      ignition::msgs::BatteryState::DISCHARGING)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }
  else if (ign_msg.power_supply_status() ==
      ignition::msgs::BatteryState::NOT_CHARGING)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  }
  else if (ign_msg.power_supply_status() ==
      ignition::msgs::BatteryState::FULL)
  {
    ros_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
  }
  else
  {
    ROS_ERROR_STREAM("Unsupported power supply status ["
              << ign_msg.power_supply_status() << "]" << std::endl);
  }

  ros_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  ros_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  ros_msg.present = true;
}

}  // namespace ros_ign_bridge
