#pragma once

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Header.h>

#include <pybind11/pybind11.h>

static inline bool is_ros_msg_type(pybind11::handle src,
                                   const std::string& msg_type_name)
{
  if (!pybind11::hasattr(src, "_type"))
  {
    return false;
  }
  std::string msg_type(src.attr("_type").cast<std::string>());
  if (msg_type != msg_type_name)
  {
    return false;
  }
  return true;
}

namespace pybind11
{
namespace detail
{
// ros::Time
template <>
struct type_caster<ros::Time>
{
public:
  PYBIND11_TYPE_CASTER(ros::Time, _("ros::Time"));

  // python -> cpp
  bool load(handle src, bool)
  {
    PyObject* source = src.ptr();
    if (!PyObject_HasAttrString(source, "secs"))
    {
      return false;
    }
    if (!PyObject_HasAttrString(source, "nsecs"))
    {
      return false;
    }

    value.sec = (src.attr("secs")).cast<uint32_t>();
    value.nsec = (src.attr("nsecs")).cast<uint32_t>();
    return true;
  }

  // cpp -> python
  static handle cast(ros::Time src, return_value_policy policy, handle parent)
  {
    object rospy = module::import("rospy");
    object TimeType = rospy.attr("Time");
    object py_obj = TimeType();
    py_obj.attr("secs") = pybind11::cast(src.sec);
    py_obj.attr("nsecs") = pybind11::cast(src.nsec);
    py_obj.inc_ref();
    return py_obj;
  }
};

// std_msgs::Header
template <>
struct type_caster<std_msgs::Header>
{
public:
  PYBIND11_TYPE_CASTER(std_msgs::Header, _("std_msgs::Header"));

  // python -> cpp
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "std_msgs/Header"))
    {
      return false;
    }
    value.seq = src.attr("seq").cast<uint32_t>();
    value.stamp = src.attr("stamp").cast<ros::Time>();
    value.frame_id = src.attr("frame_id").cast<std::string>();
    return true;
  }

  // cpp -> python
  static handle cast(std_msgs::Header header, return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("std_msgs.msg._Header");
    object MsgType = mod.attr("Header");
    object msg = MsgType();
    msg.attr("seq") = pybind11::cast(header.seq);
    msg.attr("stamp") = pybind11::cast(header.stamp);
    msg.attr("frame_id") =
        pybind11::bytes(reinterpret_cast<const char*>(&header.frame_id[0]),
                        header.frame_id.size());
    msg.inc_ref();
    return msg;
  }
};

}  // namespace detail
}  // namespace pybind11
