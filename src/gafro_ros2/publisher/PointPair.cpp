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
#include <gafro_ros2/publisher/PointPair.hpp>

namespace gafro_ros
{
    PointPair::PointPair(sackmesser_ros::Interface *interface, const std::string &name)
      : sackmesser_ros::Publisher<visualization_msgs::msg::Marker, gafro::PointPair<double>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    PointPair::~PointPair() {}

    bool PointPair::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::msg::Marker PointPair::createMessage(const gafro::PointPair<double> &points) const
    {
        visualization_msgs::msg::Marker points_msg;

        points_msg.header.frame_id = config_.frame;
        points_msg.id = 0;

        points_msg.pose.orientation.w = 1.0;

        points_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        points_msg.scale.x = 0.025;
        points_msg.scale.y = 0.025;
        points_msg.scale.z = 0.025;
        points_msg.color.r = static_cast<float>(config_.color_r);
        points_msg.color.g = static_cast<float>(config_.color_g);
        points_msg.color.b = static_cast<float>(config_.color_b);
        points_msg.color.a = static_cast<float>(config_.color_a);

        points_msg.points.push_back(convertToPoint(points.getPoint1()));
        points_msg.points.push_back(convertToPoint(points.getPoint2()));

        return points_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::PointPair, "gafro_pointpair");