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

#include <gafro/algebra.hpp>
#include <gafro_ros2/conversion/Point.hpp>

namespace gafro_ros
{

    geometry_msgs::msg::Point convertToPoint(const gafro::Point<double> &point)
    {
        geometry_msgs::msg::Point point_msg;

        point_msg.x = point.get<gafro::blades::e1>();
        point_msg.y = point.get<gafro::blades::e2>();
        point_msg.z = point.get<gafro::blades::e3>();

        return point_msg;
    }

}  // namespace gafro_ros