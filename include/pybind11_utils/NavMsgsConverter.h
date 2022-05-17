#pragma once

#include <pybind11_utils/RosMsgsConverter.h>
#include <pybind11_utils/GeometryMsgsConverter.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

namespace pybind11
{
namespace detail
{
// nav_msgs::Odometry
template <>
struct type_caster<nav_msgs::Odometry>
{
public:
  PYBIND11_TYPE_CASTER(nav_msgs::Odometry, _("nav_msgs::Odometry"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "nav_msgs/Odometry"))
    {
      return false;
    }
    value.header = (src.attr("header")).cast<std_msgs::Header>();
    value.child_frame_id = (src.attr("child_frame_id")).cast<std::string>();
    value.pose = (src.attr("pose")).cast<geometry_msgs::PoseWithCovariance>();
    value.twist =
        (src.attr("twist")).cast<geometry_msgs::TwistWithCovariance>();
    return true;
  }

  static handle cast(nav_msgs::Odometry cpp_msg, return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("nav_msgs.msg._Odometry");
    object MsgType = mod.attr("Odometry");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("child_frame_id") = pybind11::cast(cpp_msg.child_frame_id);
    msg.attr("pose") = pybind11::cast(cpp_msg.pose);
    msg.attr("twist") = pybind11::cast(cpp_msg.twist);
    msg.inc_ref();
    return msg;
  }
};

// nav_msgs::MapMetaData
template <>
struct type_caster<nav_msgs::MapMetaData>
{
public:
  PYBIND11_TYPE_CASTER(nav_msgs::MapMetaData, _("nav_msgs::MapMetaData"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "nav_msgs/MapMetaData"))
    {
      return false;
    }
    value.map_load_time = (src.attr("map_load_time")).cast<ros::Time>();
    value.resolution = (src.attr("resolution")).cast<float>();
    value.width = (src.attr("width")).cast<uint32_t>();
    value.height = (src.attr("height")).cast<uint32_t>();
    value.origin = (src.attr("origin")).cast<geometry_msgs::Pose>();
    return true;
  }

  static handle cast(nav_msgs::MapMetaData cpp_msg, return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("nav_msgs.msg._MapMetaData");
    object MsgType = mod.attr("MapMetaData");
    object msg = MsgType();
    msg.attr("map_load_time") = pybind11::cast(cpp_msg.map_load_time);
    msg.attr("resolution") = pybind11::cast(cpp_msg.resolution);
    msg.attr("width") = pybind11::cast(cpp_msg.width);
    msg.attr("height") = pybind11::cast(cpp_msg.height);
    msg.attr("origin") = pybind11::cast(cpp_msg.origin);
    msg.inc_ref();
    return msg;
  }
};

// nav_msgs::OccupancyGrid
template <>
struct type_caster<nav_msgs::OccupancyGrid>
{
public:
  PYBIND11_TYPE_CASTER(nav_msgs::OccupancyGrid, _("nav_msgs::OccupancyGrid"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "nav_msgs/OccupancyGrid"))
    {
      return false;
    }
    value.header = (src.attr("header")).cast<std_msgs::Header>();
    value.info = (src.attr("info")).cast<nav_msgs::MapMetaData>();
    value.data = (src.attr("data")).cast<std::vector<int8_t>>();
    return true;
  }

  static handle cast(nav_msgs::OccupancyGrid cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("nav_msgs.msg._OccupancyGrid");
    object MsgType = mod.attr("OccupancyGrid");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("info") = pybind11::cast(cpp_msg.info);
    msg.attr("data") = pybind11::cast(cpp_msg.data);
    msg.inc_ref();
    return msg;
  }
};
}  // namespace detail
}  // namespace pybind11
