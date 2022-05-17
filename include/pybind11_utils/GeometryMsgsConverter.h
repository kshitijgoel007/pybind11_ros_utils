#pragma once

#include <pybind11_utils/RosMsgsConverter.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

namespace pybind11
{
namespace detail
{
// geometry_msgs::Point
template <>
struct type_caster<geometry_msgs::Point>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Point, _("geometry_msgs::Point"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Point"))
    {
      return false;
    }
    value.x = (src.attr("x")).cast<double>();
    value.y = (src.attr("y")).cast<double>();
    value.z = (src.attr("z")).cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Point pt, return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Point");
    object MsgType = mod.attr("Point");
    object msg = MsgType();
    msg.attr("x") = pybind11::cast(pt.x);
    msg.attr("y") = pybind11::cast(pt.y);
    msg.attr("z") = pybind11::cast(pt.z);
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::Point32
template <>
struct type_caster<geometry_msgs::Point32>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Point32, _("geometry_msgs::Point32"));

  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Point32"))
    {
      return false;
    }
    value.x = (src.attr("x")).cast<float>();
    value.y = (src.attr("y")).cast<float>();
    value.z = (src.attr("z")).cast<float>();
    return true;
  }

  static handle cast(geometry_msgs::Point32 pt, return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Point32");
    object MsgType = mod.attr("Point32");
    object msg = MsgType();
    msg.attr("x") = pybind11::cast(pt.x);
    msg.attr("y") = pybind11::cast(pt.y);
    msg.attr("z") = pybind11::cast(pt.z);
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::Vector3
template <>
struct type_caster<geometry_msgs::Vector3>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Vector3, _("geometry_msgs::Vector3"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Vector3"))
    {
      return false;
    }
    value.x = (src.attr("x")).cast<double>();
    value.y = (src.attr("y")).cast<double>();
    value.z = (src.attr("z")).cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Vector3 cpp_msg, return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Vector3");
    object MsgType = mod.attr("Vector3");
    object msg = MsgType();
    msg.attr("x") = pybind11::cast(cpp_msg.x);
    msg.attr("y") = pybind11::cast(cpp_msg.y);
    msg.attr("z") = pybind11::cast(cpp_msg.z);
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::Quaternion
template <>
struct type_caster<geometry_msgs::Quaternion>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Quaternion, _("geometry_msgs::"
                                                    "Quaternion"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Quaternion"))
    {
      return false;
    }
    value.x = (src.attr("x")).cast<double>();
    value.y = (src.attr("y")).cast<double>();
    value.z = (src.attr("z")).cast<double>();
    value.w = (src.attr("w")).cast<double>();
    return true;
  }

  static handle cast(geometry_msgs::Quaternion cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Quaternion");
    object MsgType = mod.attr("Quaternion");
    object msg = MsgType();
    msg.attr("x") = pybind11::cast(cpp_msg.x);
    msg.attr("y") = pybind11::cast(cpp_msg.y);
    msg.attr("z") = pybind11::cast(cpp_msg.z);
    msg.attr("w") = pybind11::cast(cpp_msg.w);
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::Transform
template <>
struct type_caster<geometry_msgs::Transform>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Transform, _("geometry_msgs::Transform"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Transform"))
    {
      return false;
    }
    value.translation =
        (src.attr("translation")).cast<geometry_msgs::Vector3>();
    value.rotation = (src.attr("rotation")).cast<geometry_msgs::Quaternion>();
    return true;
  }

  static handle cast(geometry_msgs::Transform cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Transform");
    object MsgType = mod.attr("Transform");
    object msg = MsgType();
    msg.attr("translation") = pybind11::cast(cpp_msg.translation);
    msg.attr("rotation") = pybind11::cast(cpp_msg.rotation);
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::TransformStamped
template <>
struct type_caster<geometry_msgs::TransformStamped>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::TransformStamped, _("geometry_msgs::"
                                                          "TransformStamped"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/TransformStamped"))
    {
      return false;
    }
    value.header = src.attr("header").cast<std_msgs::Header>();
    value.child_frame_id = src.attr("child_frame_id").cast<std::string>();
    value.transform = src.attr("transform").cast<geometry_msgs::Transform>();
    return true;
  }

  static handle cast(geometry_msgs::TransformStamped cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("geometry_msgs.msg._TransformStamped");
    object MsgType = mod.attr("TransformStamped");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    // msg.attr("child_frame_id") = pybind11::cast(cpp_msg.child_frame_id);
    msg.attr("child_frame_id") = pybind11::bytes(
        reinterpret_cast<const char*>(&cpp_msg.child_frame_id[0]),
        cpp_msg.child_frame_id.size());
    msg.attr("transform") = pybind11::cast(cpp_msg.transform);
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::Pose
template <>
struct type_caster<geometry_msgs::Pose>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Pose, _("geometry_msgs::Pose"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Pose"))
    {
      return false;
    }
    value.position = (src.attr("position")).cast<geometry_msgs::Point>();
    value.orientation =
        (src.attr("orientation")).cast<geometry_msgs::Quaternion>();
    return true;
  }

  static handle cast(geometry_msgs::Pose cpp_msg, return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Pose");
    object MsgType = mod.attr("Pose");
    object msg = MsgType();
    msg.attr("position") = pybind11::cast(cpp_msg.position);
    msg.attr("orientation") = pybind11::cast(cpp_msg.orientation);
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::PoseWithCovariance
// WARNING: float64 is a double in C++ and float in python
// http://wiki.ros.org/msg#Fields
template <>
struct type_caster<geometry_msgs::PoseWithCovariance>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::PoseWithCovariance, _("geometry_msgs::"
                                                            "PoseWithCovarianc"
                                                            "e"));

  // python -> cpp
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/PoseWithCovariance"))
    {
      return false;
    }
    value.pose = (src.attr("pose")).cast<geometry_msgs::Pose>();
    pybind11::list cov_lst = (src.attr("covariance")).cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(cov_lst); ++i)
    {
      double pf((cov_lst[i]).cast<double>());
      value.covariance[pf];
    }
    return true;
  }

  // cpp -> python
  static handle cast(geometry_msgs::PoseWithCovariance cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("geometry_msgs.msg._PoseWithCovariance");
    object MsgType = mod.attr("PoseWithCovariance");
    object msg = MsgType();
    msg.attr("pose") = pybind11::cast(cpp_msg.pose);
    pybind11::list cov_lst;
    for (size_t i = 0; i < cpp_msg.covariance.size(); ++i)
    {
      const float& pf(cpp_msg.covariance[i]);
      cov_lst[i] = pybind11::cast(pf);
    }
    msg.attr("covariance") = cov_lst;
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::Twist
template <>
struct type_caster<geometry_msgs::Twist>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::Twist, _("geometry_msgs::Twist"));

  // python -> cpp
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/Twist"))
    {
      return false;
    }
    value.linear = (src.attr("linear")).cast<geometry_msgs::Vector3>();
    value.angular = (src.attr("angular")).cast<geometry_msgs::Vector3>();
    return true;
  }

  // cpp -> python
  static handle cast(geometry_msgs::Twist cpp_msg, return_value_policy policy,
                     handle parent)
  {
    object mod = module::import("geometry_msgs.msg._Twist");
    object MsgType = mod.attr("Twist");
    object msg = MsgType();
    msg.attr("linear") = pybind11::cast(cpp_msg.linear);
    msg.attr("angular") = pybind11::cast(cpp_msg.angular);
    msg.inc_ref();
    return msg;
  }
};

// geometry_msgs::TwistWithCovariance
// WARNING: float64 is a double in C++ and float in python
// http://wiki.ros.org/msg#Fields
template <>
struct type_caster<geometry_msgs::TwistWithCovariance>
{
public:
  PYBIND11_TYPE_CASTER(geometry_msgs::TwistWithCovariance, _("geometry_msgs::"
                                                             "TwistWithCovarian"
                                                             "ce"));

  // python -> cpp
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "geometry_msgs/TwistWithCovariance"))
    {
      return false;
    }
    value.twist = (src.attr("twist")).cast<geometry_msgs::Twist>();
    pybind11::list cov_lst = (src.attr("covariance")).cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(cov_lst); ++i)
    {
      double pf((cov_lst[i]).cast<double>());
      value.covariance[i] = pf;
    }
    return true;
  }

  // cpp -> python
  static handle cast(geometry_msgs::TwistWithCovariance cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("geometry_msgs.msg._TwistWithCovariance");
    object MsgType = mod.attr("TwistWithCovariance");
    object msg = MsgType();
    msg.attr("twist") = pybind11::cast(cpp_msg.twist);
    pybind11::list cov_lst;
    for (size_t i = 0; i < cpp_msg.covariance.size(); ++i)
    {
      const float& pf(cpp_msg.covariance[i]);
      cov_lst[i] = pybind11::cast(pf);
    }
    msg.attr("covariance") = cov_lst;
    msg.inc_ref();
    return msg;
  }
};

}  // namespace detail
}  // namespace pybind11
