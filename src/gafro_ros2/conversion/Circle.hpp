/*
    Copyright (c) Idiap Research Institute, http://www.idiap.ch/
    Written by Tobias Löw <https://tobiloew.ch>

    This file is part of gafro_ros.

    gafro is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3 as
    published by the Free Software Foundation.

    gafro is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with gafro. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <gafro/algebra/cga/Circle.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace gafro_ros
{

    visualization_msgs::msg::Marker convertToMarker(const gafro::Circle<double> &circle, const std::string &frame,
                                                    const int &id,  //
                                                    const double &r = 1.0, const double &g = 0.0, const double &b = 0.0, const double &a = 1.0);

}  // namespace gafro_ros