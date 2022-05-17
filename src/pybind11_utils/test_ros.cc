/*
  BSD 3-Clause License

  Copyright (c) 2020, Kshitij Goel
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <pybind11_utils/pybind11_utils.h>

namespace py = pybind11;

void print_time(const ros::Time &t) {
  ROS_INFO("Printing time in C++");
  std::cerr << "seconds: " << t.sec;
  std::cerr << " nanoseconds: " << t.nsec << std::endl;
}

ros::Time get_arbitrary_time() {
  ROS_INFO("Making up some arbitrary time from C++");
  ros::Time ts;
  ts.sec = 2692;
  ts.nsec = 9999999;
  print_time(ts);
  return ts;
}

void print_header(const std_msgs::Header &h) {
  ROS_INFO("Printing header in C++");
  std::cerr << "seq: " << h.seq << std::endl;
  print_time(h.stamp);
  std::cerr << "frame_id: " << h.frame_id << std::endl;
}

std_msgs::Header get_arbitrary_header() {
  ROS_INFO("Making up some arbitrary header from C++");
  ros::Time ts;
  ts.sec = 2692;
  ts.nsec = 9999999;
  std_msgs::Header h;
  h.stamp = ts;
  h.frame_id = "world";
  return h;
}

PYBIND11_MODULE(test_ros, m) {
  m.doc() = "module to test ros topic converters";
  m.def("print_time", &print_time, "print the supplied time");
  m.def("get_arbitrary_time", &get_arbitrary_time,
        "get some arbitrary time from C++");
  m.def("print_header", &print_header, "print the header message");
  m.def("get_arbitrary_header", &get_arbitrary_header,
        "get some arbitrary header from C++");
}
