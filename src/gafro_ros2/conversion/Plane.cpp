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

#include <Eigen/Geometry>
#include <gafro/algebra.hpp>
#include <gafro_ros2/conversion/Plane.hpp>
#include <gafro_ros2/conversion/Point.hpp>

namespace gafro_ros
{

    visualization_msgs::msg::Marker convertToMarker(const gafro::Plane<double> &plane)
    {
        visualization_msgs::msg::Marker plane_msg;

        gafro::Motor<double> motor = gafro::Plane<double>::XY().getMotor(plane);

        gafro::Point<double> p0 = motor.apply(gafro::Point<double>());
        gafro::Point<double> p1 = motor.apply(gafro::Point<double>(1.0, 0.0, 0.0));
        gafro::Point<double> p2 = motor.apply(gafro::Point<double>(0.0, 1.0, 0.0));
        gafro::Point<double> p3 = motor.apply(gafro::Point<double>(-1.0, 0.0, 0.0));
        gafro::Point<double> p4 = motor.apply(gafro::Point<double>(0.0, -1.0, 0.0));

        plane_msg.points.push_back(convertToPoint(p1));
        plane_msg.points.push_back(convertToPoint(p2));
        plane_msg.points.push_back(convertToPoint(p0));

        plane_msg.points.push_back(convertToPoint(p2));
        plane_msg.points.push_back(convertToPoint(p3));
        plane_msg.points.push_back(convertToPoint(p0));

        plane_msg.points.push_back(convertToPoint(p3));
        plane_msg.points.push_back(convertToPoint(p4));
        plane_msg.points.push_back(convertToPoint(p0));

        plane_msg.points.push_back(convertToPoint(p4));
        plane_msg.points.push_back(convertToPoint(p1));
        plane_msg.points.push_back(convertToPoint(p0));

        plane_msg.pose.orientation.w = 1.0;

        plane_msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        plane_msg.scale.x = 1.0;
        plane_msg.scale.y = 1.0;
        plane_msg.scale.z = 1.0;

        return plane_msg;
    }

}  // namespace gafro_ros