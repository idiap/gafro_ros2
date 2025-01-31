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
#include <gafro_ros2/conversion/Circle.hpp>
#include <gafro_ros2/conversion/Point.hpp>

namespace gafro_ros
{

    visualization_msgs::msg::Marker convertToMarker(const gafro::Circle<double> &circle, const std::string &frame, const int &id,  //
                                                    const double &r, const double &g, const double &b, const double &a)
    {
        visualization_msgs::msg::Marker circle_msg;

        circle_msg.header.frame_id = frame;
        // circle_msg.header.stamp = ros::Time::now();
        circle_msg.id = id;

        double radius = circle.getRadius();

        gafro::Plane<double> plane = circle.getPlane();
        plane.normalize();

        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d({ 0.0, 0.0, 1.0 }), plane.getNormal().vector()).normalized();

        circle_msg.pose.position = convertToPoint(circle.getCenter());
        circle_msg.pose.orientation.x = q.x();
        circle_msg.pose.orientation.y = q.y();
        circle_msg.pose.orientation.z = q.z();
        circle_msg.pose.orientation.w = q.w();

        circle_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
        circle_msg.scale.x = 0.002;
        circle_msg.color.r = static_cast<float>(r);
        circle_msg.color.g = static_cast<float>(g);
        circle_msg.color.b = static_cast<float>(b);
        circle_msg.color.a = static_cast<float>(a);

        for (int k = 0; k < 101; ++k)
        {
            double angle = 2.0 * M_PI * k / 100.0;
            geometry_msgs::msg::Point point;
            point.x = radius * std::cos(angle);
            point.y = radius * std::sin(angle);
            point.z = 0.0;
            circle_msg.points.push_back(point);
        }

        return circle_msg;
    }

}  // namespace gafro_ros