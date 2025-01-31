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
#include <gafro_ros2/conversion/Plane.hpp>
//
#include <gafro_ros2/publisher/PlaneVector.hpp>

namespace gafro_ros
{
    PlaneVector::PlaneVector(sackmesser_ros::Interface *interface, const std::string &name)
      : sackmesser_ros::Publisher<visualization_msgs::msg::MarkerArray, std::vector<gafro::Plane<double>>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    PlaneVector::~PlaneVector() {}

    bool PlaneVector::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::msg::MarkerArray PlaneVector::createMessage(const std::vector<gafro::Plane<double>> &planes) const
    {
        visualization_msgs::msg::MarkerArray planes_msg;

        int id = 0;

        for (const auto &plane : planes)
        {
            visualization_msgs::msg::Marker plane_msg = convertToMarker(plane);

            plane_msg.id = ++id;

            plane_msg.header.frame_id = config_.frame;

            plane_msg.color.r = static_cast<float>(config_.color_r);
            plane_msg.color.g = static_cast<float>(config_.color_g);
            plane_msg.color.b = static_cast<float>(config_.color_b);
            plane_msg.color.a = static_cast<float>(config_.color_a);

            planes_msg.markers.push_back(plane_msg);
        }

        return planes_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::PlaneVector, "gafro_plane_vector");