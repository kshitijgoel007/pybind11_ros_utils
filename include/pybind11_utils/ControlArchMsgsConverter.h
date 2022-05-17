#pragma once

#include <pybind11_utils/GeometryMsgsConverter.h>

#include <control_arch/utils/state_t.h>

namespace pybind11 {
namespace detail {

// control_arch::State
template <> struct type_caster<control_arch::State> {
public:
  PYBIND11_TYPE_CASTER(control_arch::State,
                       _("control_arch::State"));

  // python -> cpp
  bool load(handle src, bool) {
    if (!is_ros_msg_type(src, "control_arch/State")) {
      return false;
    }
    value.pos = (src.attr("pos")).cast<geometry_msgs::Point>();
    value.vel = (src.attr("vel")).cast<geometry_msgs::Vector3>();
    value.acc = (src.attr("acc")).cast<geometry_msgs::Vector3>();
    value.jerk = (src.attr("jerk")).cast<geometry_msgs::Vector3>();
    value.snap = (src.attr("snap")).cast<geometry_msgs::Vector3>();
    value.rot = (src.attr("rot")).cast<geometry_msgs::Quaternion>();
    value.ang = (src.attr("ang")).cast<geometry_msgs::Vector3>();
    value.angacc = (src.attr("angacc")).cast<geometry_msgs::Vector3>();
    value.yaw = (src.attr("yaw")).cast<float>();
    value.yawdot = (src.attr("yawdot")).cast<float>();
    value.yawddot = (src.attr("yawddot")).cast<float>();
    return true;
  }

  // cpp -> python
  static handle cast(control_arch::State cpp_msg,
                     return_value_policy policy, handle parent) {
    object mod = module::import("control_arch.msg._State");
    object MsgType = mod.attr("State");
    object msg = MsgType();
    msg.attr("pos") = pybind11::cast(cpp_msg.pos);
    msg.attr("vel") = pybind11::cast(cpp_msg.vel);
    msg.attr("acc") = pybind11::cast(cpp_msg.acc);
    msg.attr("jerk") = pybind11::cast(cpp_msg.jerk);
    msg.attr("snap") = pybind11::cast(cpp_msg.snap);

    msg.attr("rot") = pybind11::cast(cpp_msg.rot);
    msg.attr("ang") = pybind11::cast(cpp_msg.ang);
    msg.attr("angacc") = pybind11::cast(cpp_msg.angacc);

    msg.attr("yaw") = pybind11::cast(cpp_msg.yaw);
    msg.attr("yawdot") = pybind11::cast(cpp_msg.yawdot);
    msg.attr("yawddot") = pybind11::cast(cpp_msg.yawddot);

    msg.inc_ref();
    return msg;
  }
};

} // namespace detail
} // namespace pybind11
