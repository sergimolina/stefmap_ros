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

#pragma once

#include <array>
#include <stdint.h>
#include <vector>

#include <ros/console.h>
#include <ros/ros.h>

#include <stefmap_ros/GetSTeFMap.h>
#include <stefmap_ros/STeFMapMsg.h>

#include <std_msgs/Header.h>

namespace stefmap_ros {

struct STeFMapCell {
  size_t row;
  size_t column;
  double x;
  double y;
  double best_angle;
  std::array<double, 8> probabilities;

  STeFMapCell();
  STeFMapCell(const STeFMapCellMsg &cell_msg);
};

class STeFMap {
  std_msgs::Header header;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double cell_size_;
  size_t rows_;
  size_t columns_;
  std::vector<STeFMapCellMsg> cells_;

public:
  inline double getPredictionTime() const { return header.stamp.toSec(); }
  inline std::string getFrameId() const { return header.frame_id; }
  inline double getXMin() const { return x_min_; }
  inline double getXMax() const { return x_max_; }
  inline double getYMin() const { return y_min_; }
  inline double getYMax() const { return y_max_; }
  inline size_t getRows() const { return rows_; }
  inline size_t getColumns() const { return columns_; }
  inline double getCellSize() const { return cell_size_; }

  inline std::vector<STeFMapCellMsg> getCell() const { return cells_; }
  inline size_t x2index(double x) const {
    return std::round((x - x_min_) / cell_size_);
  }

  inline size_t y2index(double y) const {
    return std::round((y - y_min_) / cell_size_);
  }

  bool isOrganized() const;

  STeFMapCell at(size_t row, size_t col) const;
  STeFMapCell operator()(double x, double y) const;

  STeFMap(const STeFMapMsg &stefmap_msg);

  virtual ~STeFMap() {}
};

class STeFMapClient {
  ros::NodeHandle nh;
  ros::ServiceClient stefmap_client;

public:
  STeFMapClient();

  STeFMapMsg get(double prediction_time, int order);
};

typedef std::shared_ptr<STeFMap> STeFMapPtr;
typedef std::shared_ptr<const STeFMap> STeFMapConstPtr;

} /* namespace stefmap_ros */
