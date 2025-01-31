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
#include <gafro_ros2/conversion/Circle.hpp>
//
#include <gafro_ros2/publisher/CircleVector.hpp>

namespace gafro_ros
{
    CircleVector::CircleVector(sackmesser_ros::Interface *interface, const std::string &name)
      : sackmesser_ros::Publisher<visualization_msgs::msg::MarkerArray, std::vector<gafro::Circle<double>>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    CircleVector::~CircleVector() {}

    bool CircleVector::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::msg::MarkerArray CircleVector::createMessage(const std::vector<gafro::Circle<double>> &circles) const
    {
        visualization_msgs::msg::MarkerArray circles_msg;

        int i = 0;

        for (const auto &circle : circles)
        {
            visualization_msgs::msg::Marker circle_msg =
              convertToMarker(circle, config_.frame, ++i, config_.color_r, config_.color_g, config_.color_b, config_.color_a);

            // circle_msg.header.frame_id = config_.frame;
            // circle_msg.id = ++i;

            // circle_msg.pose.orientation.w = 1.0;

            // circle_msg.scale.x = 0.01;
            // circle_msg.scale.y = 0.01;
            // circle_msg.scale.z = 0.01;
            // circle_msg.color.r = static_cast<float>(config_.color_r);
            // circle_msg.color.g = static_cast<float>(config_.color_g);
            // circle_msg.color.b = static_cast<float>(config_.color_b);
            // circle_msg.color.a = static_cast<float>(config_.color_a);

            circles_msg.markers.push_back(circle_msg);
        }

        return circles_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::CircleVector, "gafro_circle_vector");