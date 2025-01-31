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

#include <gafro_ros2/conversion/Circle.hpp>
#include <gafro_ros2/publisher/Circle.hpp>

namespace gafro_ros
{
    Circle::Circle(sackmesser_ros::Interface *interface, const std::string &name)  //
      : sackmesser_ros::Publisher<visualization_msgs::msg::Marker, gafro::Circle<double>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    Circle::~Circle() {}

    bool Circle::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true) &&  //
               server->loadParameter(ns + "thickness", &thickness, true);
    }

    visualization_msgs::msg::Marker Circle::createMessage(const gafro::Circle<double> &circle) const
    {
        visualization_msgs::msg::Marker marker =
          convertToMarker(circle, config_.frame, 0, config_.color_r, config_.color_g, config_.color_b, config_.color_a);

        marker.scale.x = config_.thickness;

        return marker;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::Circle, "gafro_circle");