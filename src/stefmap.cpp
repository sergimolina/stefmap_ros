/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   Copyright (c) Sergi Molina
 *   This file is part of stefmap_ros.
 *
 *   stefmap_ros is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public License as
 *   published by the Free Software Foundation, either version 3 of the License,
 *   or (at your option) any later version.
 *
 *   stefmap_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with cliffmap_rviz_plugin.  If not, see
 *   <https://www.gnu.org/licenses/>.
 */

#include <stefmap_ros/stefmap.hpp>

namespace stefmap_ros {

STeFMapCell::STeFMapCell()
    : row(0),
      column(0),
      x(0.0),
      y(0.0),
      best_angle(0.0),
      probabilities({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}

STeFMapCell::STeFMapCell(const STeFMapCellMsg& cell_msg) {
  row = cell_msg.row;
  column = cell_msg.column;
  x = cell_msg.x;
  y = cell_msg.y;
  best_angle = cell_msg.best_angle;
  for (size_t i = 0; i < 8; i++) {
    probabilities[i] = cell_msg.probabilities[i];
  }
}

STeFMap::STeFMap(const STeFMapMsg& stefmap_msg)
    : header(stefmap_msg.header),
      x_min_(stefmap_msg.x_min),
      x_max_(stefmap_msg.x_max),
      y_min_(stefmap_msg.y_min),
      y_max_(stefmap_msg.y_max),
      cell_size_(stefmap_msg.cell_size),
      rows_(stefmap_msg.rows),
      columns_(stefmap_msg.columns),
      cells_(stefmap_msg.cells) {
  ROS_INFO_STREAM("Read a STeF-Map with cell size: " << cell_size_ << " m.");
}

bool STeFMap::isOrganized() const {
  for (size_t r = 0; r < this->rows_; r++)
    for (size_t c = 0; c < this->columns_; c++) {
      const auto& cell = at(r, c);
      if (cell.row != r || cell.column != c) return false;
    }
  return true;
}
STeFMapCell STeFMap::operator()(double x, double y) const {
  size_t row = y2index(y);
  size_t col = x2index(x);
  return this->at(row, col);
}

STeFMapCell STeFMap::at(size_t row, size_t col) const {
  if (row >= rows_ || col >= columns_) {
    ROS_DEBUG_STREAM(
        "WARNING STeFMap::at called with out-of-bounds indices. Returning "
        "empty STeFMapCell.");
    return STeFMapCell();
  }

  return cells_[row * columns_ + col];
}

STeFMapMsg STeFMapClient::get(double prediction_time, int order) {
  GetSTeFMap msg;
  msg.request.prediction_time = prediction_time;
  msg.request.order = order;

  if (!stefmap_client.call(msg)) {
    ROS_ERROR_STREAM(
        "Failed to call STeF-Map msg. Service call failed. Empty map "
        "returned.");
    return STeFMapMsg();
  }
  ROS_INFO_STREAM("Got a STeF-Map msg.");
  return msg.response.stefmap;
}

STeFMapClient::STeFMapClient() {
  stefmap_client = nh.serviceClient<GetSTeFMap>("get_stefmap");
  stefmap_client.waitForExistence();
  ROS_INFO_STREAM("Connected to STeF-Map server.");
}

} /* namespace stefmap_ros */
