#pragma once

#include <pybind11_utils/RosMsgsConverter.h>
#include <pybind11/stl.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

namespace pybind11
{
namespace detail
{
// sensor_msgs::PointField
template <>
struct type_caster<sensor_msgs::PointField>
{
public:
  PYBIND11_TYPE_CASTER(sensor_msgs::PointField, _("sensor_msgs::PointField"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/PointField"))
    {
      return false;
    }
    value.name = (src.attr("name")).cast<std::string>();
    value.offset = (src.attr("offset")).cast<uint32_t>();
    value.datatype = (src.attr("datatype")).cast<uint8_t>();
    value.count = (src.attr("count")).cast<uint32_t>();
    return true;
  }

  static handle cast(sensor_msgs::PointField cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("sensor_msgs.msg._PointField");
    object MsgType = mod.attr("PointField");
    object msg = MsgType();
    // avoid !!python/unicode problem.
    // msg.attr("name") = pybind11::cast(cpp_msg.name);
    // msg.attr("name") = PyString_FromString(cpp_msg.name.c_str());
    msg.attr("name") = pybind11::bytes(
        reinterpret_cast<const char*>(&cpp_msg.name[0]), cpp_msg.name.size());
    msg.attr("offset") = pybind11::cast(cpp_msg.offset);
    msg.attr("datatype") = pybind11::cast(cpp_msg.datatype);
    msg.attr("count") = pybind11::cast(cpp_msg.count);
    msg.inc_ref();
    return msg;
  }
};

// sensor_msgs::PointCloud2
template <>
struct type_caster<sensor_msgs::PointCloud2>
{
public:
  PYBIND11_TYPE_CASTER(sensor_msgs::PointCloud2, _("sensor_msgs::PointCloud2"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/PointCloud2"))
    {
      return false;
    }
    value.header = (src.attr("header")).cast<std_msgs::Header>();
    value.height = (src.attr("height")).cast<uint32_t>();
    value.width = (src.attr("width")).cast<uint32_t>();
    pybind11::list field_lst = (src.attr("fields")).cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(field_lst); ++i)
    {
      sensor_msgs::PointField pf(
          (field_lst[i]).cast<sensor_msgs::PointField>());
      value.fields.push_back(pf);
    }
    value.is_bigendian = (src.attr("is_bigendian")).cast<bool>();
    value.point_step = (src.attr("point_step")).cast<uint32_t>();
    value.row_step = (src.attr("row_step")).cast<uint32_t>();
    std::string data_str = (src.attr("data")).cast<std::string>();
    value.data.insert(value.data.end(), data_str.c_str(),
                      data_str.c_str() + data_str.length());
    value.is_dense = (src.attr("is_dense")).cast<bool>();
    return true;
  }

  static handle cast(sensor_msgs::PointCloud2 cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("sensor_msgs.msg._PointCloud2");
    object MsgType = mod.attr("PointCloud2");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("height") = pybind11::cast(cpp_msg.height);
    msg.attr("width") = pybind11::cast(cpp_msg.width);
    // msg.attr("fields") = pybind11::cast(cpp_msg.fields);
    // pybind11::list field_lst = (msg.attr("fields")).cast<pybind11::list>();
    pybind11::list field_lst;
    for (size_t i = 0; i < cpp_msg.fields.size(); ++i)
    {
      const sensor_msgs::PointField& pf(cpp_msg.fields[i]);
      field_lst.append(pybind11::cast(pf));
    }
    msg.attr("fields") = field_lst;
    msg.attr("is_bigendian") = pybind11::cast(cpp_msg.is_bigendian);
    msg.attr("point_step") = pybind11::cast(cpp_msg.point_step);
    msg.attr("row_step") = pybind11::cast(cpp_msg.row_step);
    // msg.attr("data") = pybind11::bytes(std::string(cpp_msg.data.begin(),
    // cpp_msg.data.end()));
    msg.attr("data") = pybind11::bytes(
        reinterpret_cast<const char*>(&cpp_msg.data[0]), cpp_msg.data.size());
    msg.attr("is_dense") = pybind11::cast(cpp_msg.is_dense);
    msg.inc_ref();
    return msg;
  }
};

// sensor_msgs::ChannelFloat32
template <>
struct type_caster<sensor_msgs::ChannelFloat32>
{
public:
  PYBIND11_TYPE_CASTER(sensor_msgs::ChannelFloat32, _("sensor_msgs::"
                                                      "ChannelFloat32"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/ChannelFloat32"))
    {
      return false;
    }

    value.name = (src.attr("name")).cast<std::string>();

    pybind11::list values_lst = (src.attr("values")).cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(values_lst); ++i)
    {
      float pf((values_lst[i]).cast<float>());
      value.values.push_back(pf);
    }

    return true;
  }

  static handle cast(sensor_msgs::ChannelFloat32 cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("sensor_msgs.msg._ChannelFloat32");
    object MsgType = mod.attr("ChannelFloat32");
    object msg = MsgType();
    msg.attr("name") = pybind11::cast(cpp_msg.name);
    pybind11::list values_lst;
    for (size_t i = 0; i < cpp_msg.values.size(); ++i)
    {
      const float& pf(cpp_msg.values[i]);
      values_lst.append(pybind11::cast(pf));
    }
    msg.attr("values") = values_lst;
    msg.inc_ref();
    return msg;
  }
};

// sensor_msgs::PointCloud
template <>
struct type_caster<sensor_msgs::PointCloud>
{
public:
  PYBIND11_TYPE_CASTER(sensor_msgs::PointCloud, _("sensor_msgs::PointCloud"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/PointCloud"))
    {
      return false;
    }

    value.header = (src.attr("header")).cast<std_msgs::Header>();

    pybind11::list points_lst = (src.attr("points")).cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(points_lst); ++i)
    {
      geometry_msgs::Point32 pf((points_lst[i]).cast<geometry_msgs::Point32>());
      value.points.push_back(pf);
    }

    pybind11::list channels_lst = (src.attr("channels")).cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(channels_lst); ++i)
    {
      sensor_msgs::ChannelFloat32 pf(
          (channels_lst[i]).cast<sensor_msgs::ChannelFloat32>());
      value.channels.push_back(pf);
    }

    return true;
  }

  static handle cast(sensor_msgs::PointCloud cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("sensor_msgs.msg._PointCloud");
    object MsgType = mod.attr("PointCloud");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    pybind11::list points_lst;
    for (size_t i = 0; i < cpp_msg.points.size(); ++i)
    {
      const geometry_msgs::Point32& pf(cpp_msg.points[i]);
      points_lst.append(pybind11::cast(pf));
    }
    msg.attr("points") = points_lst;

    pybind11::list channels_lst;
    for (size_t i = 0; i < cpp_msg.channels.size(); ++i)
    {
      const sensor_msgs::ChannelFloat32& pf(cpp_msg.channels[i]);
      channels_lst.append(pybind11::cast(pf));
    }
    msg.attr("channels") = channels_lst;

    msg.inc_ref();
    return msg;
  }
};

// sensor_msgs::LaserScan
template <>
struct type_caster<sensor_msgs::LaserScan>
{
public:
  PYBIND11_TYPE_CASTER(sensor_msgs::LaserScan, _("sensor_msgs::LaserScan"));
  bool load(handle src, bool)
  {
    if (!is_ros_msg_type(src, "sensor_msgs/LaserScan"))
    {
      return false;
    }

    value.header = (src.attr("header")).cast<std_msgs::Header>();
    value.angle_min = (src.attr("angle_min")).cast<float>();
    value.angle_max = (src.attr("angle_max")).cast<float>();
    value.angle_increment = (src.attr("angle_increment")).cast<float>();
    value.time_increment = (src.attr("time_increment")).cast<float>();
    value.scan_time = (src.attr("scan_time")).cast<float>();
    value.range_min = (src.attr("range_min")).cast<float>();
    value.range_max = (src.attr("range_max")).cast<float>();

    pybind11::list ranges_lst = (src.attr("ranges")).cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(ranges_lst); ++i)
    {
      float pf((ranges_lst[i]).cast<float>());
      value.ranges.push_back(pf);
    }

    pybind11::list intensities_lst = (src.attr("intensities")).cast<pybind11::list>();
    for (int i = 0; i < pybind11::len(intensities_lst); ++i)
    {
      float pf((intensities_lst[i]).cast<float>());
      value.intensities.push_back(pf);
    }

    return true;
  }

  static handle cast(sensor_msgs::LaserScan cpp_msg,
                     return_value_policy policy, handle parent)
  {
    object mod = module::import("sensor_msgs.msg._LaserScan");
    object MsgType = mod.attr("LaserScan");
    object msg = MsgType();
    msg.attr("header") = pybind11::cast(cpp_msg.header);
    msg.attr("angle_min") = pybind11::cast(cpp_msg.angle_min);
    msg.attr("angle_max") = pybind11::cast(cpp_msg.angle_max);
    msg.attr("angle_increment") = pybind11::cast(cpp_msg.angle_increment);
    msg.attr("time_increment") = pybind11::cast(cpp_msg.time_increment);
    msg.attr("scan_time") = pybind11::cast(cpp_msg.scan_time);
    msg.attr("range_min") = pybind11::cast(cpp_msg.range_min);
    msg.attr("range_max") = pybind11::cast(cpp_msg.range_max);
    pybind11::list ranges_lst;
    for (size_t i = 0; i < cpp_msg.ranges.size(); ++i)
    {
      const float& pf(cpp_msg.ranges[i]);
      ranges_lst.append(pybind11::cast(pf));
    }
    msg.attr("ranges") = ranges_lst;

    pybind11::list intensities_lst;
    for (size_t i = 0; i < cpp_msg.intensities.size(); ++i)
    {
      const float& pf(cpp_msg.intensities[i]);
      intensities_lst.append(pybind11::cast(pf));
    }
    msg.attr("intensities") = intensities_lst;

    msg.inc_ref();
    return msg;
  }
};


}  // namespace detail
}  // namespace pybind11
