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
//
#include <gafro_ros2/publisher/Point.hpp>

namespace gafro_ros
{
    Point::Point(sackmesser_ros::Interface *interface, const std::string &name)
      : sackmesser_ros::Publisher<visualization_msgs::msg::Marker, gafro::Point<double>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    Point::~Point() {}

    bool Point::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::msg::Marker Point::createMessage(const gafro::Point<double> &point) const
    {
        visualization_msgs::msg::Marker point_msg;

        point_msg.points.push_back(convertToPoint(point));

        point_msg.header.frame_id = config_.frame;
        point_msg.id = 0;

        point_msg.pose.orientation.w = 1.0;

        point_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        point_msg.scale.x = 0.025;
        point_msg.scale.y = 0.025;
        point_msg.scale.z = 0.025;
        point_msg.color.r = static_cast<float>(config_.color_r);
        point_msg.color.g = static_cast<float>(config_.color_g);
        point_msg.color.b = static_cast<float>(config_.color_b);
        point_msg.color.a = static_cast<float>(config_.color_a);

        return point_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::Point, "gafro_point");