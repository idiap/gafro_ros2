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
//
#include <gafro_ros2/conversion/Point.hpp>
#include <gafro_ros2/conversion/Sphere.hpp>

namespace gafro_ros
{

    visualization_msgs::msg::Marker convertToMarker(const gafro::Sphere<double> &sphere)
    {
        visualization_msgs::msg::Marker sphere_msg;

        sphere_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;

        sphere_msg.points.push_back(convertToPoint(sphere.getCenter()));

        double diameter = 2.0 * sphere.getRadius();

        sphere_msg.pose.orientation.w = 1.0;

        sphere_msg.scale.x = diameter;
        sphere_msg.scale.y = diameter;
        sphere_msg.scale.z = diameter;

        return sphere_msg;
    }

}  // namespace gafro_ros