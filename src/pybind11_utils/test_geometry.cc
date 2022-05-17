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

#include <fstream>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>

#include <pybind11/stl.h>
#include <pybind11_utils/pybind11_utils.h>

#include <eigen_utils/EigenUtilsROS.h>

namespace py = pybind11;

template <typename T, typename M> T load_txt(const std::string &path) {
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<M> values;
  uint rows = 0;
  while (std::getline(indata, line)) {
    std::stringstream line_stream(line);
    std::string cell;
    while (std::getline(line_stream, cell, ' ')) {
      values.push_back(std::stod(cell));
    }
    ++rows;
  }
  return Eigen::Map<
      const Eigen::Matrix<typename T::Scalar, T::RowsAtCompileTime,
                          T::ColsAtCompileTime, Eigen::ColMajor>>(
      values.data(), values.size() / rows, rows);
}

void print_point(const geometry_msgs::Point &p) {
  std::cerr << "x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
}

std::vector<geometry_msgs::Point> get_points() {
  Eigen::MatrixXf point_data = load_txt<Eigen::MatrixXf, float>(
      "/home/thor/sandbox/cave_exploration_sandbox/wet/src/pybind11_utils/"
      "test_scripts/geometry/files/points.txt");

  std::vector<geometry_msgs::Point> points_vector;
  for (int i = 0; i < point_data.cols(); ++i) {
    geometry_msgs::Point point;
    point.x = point_data(0, i);
    point.y = point_data(1, i);
    point.z = point_data(2, i);
    points_vector.push_back(point);
  }

  return points_vector;
}

geometry_msgs::Transform get_transforms() {
  // Displace by 1.0 along all three axes
  geometry_msgs::Transform T;
  T.translation.x = 1.0;
  T.translation.y = 1.0;
  T.translation.z = 1.0;
  T.rotation.x = 0.0;
  T.rotation.y = 0.0;
  T.rotation.z = 0.0;
  T.rotation.w = 1.0;

  return T;
}

PYBIND11_MODULE(test_geometry, m) {
  m.doc() = "module to test geometry topic converters";
  m.def("print_point", &print_point, "print the supplied point");
  m.def("get_points", &get_points,
        "get a vector of geometry_msgs::Point from txt");
  m.def("get_transforms", &get_transforms,
        "get an example affine transform");
}
