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
#include <gafro_ros2/conversion/Sphere.hpp>
//
#include <gafro_ros2/publisher/SphereVector.hpp>

namespace gafro_ros
{
    SphereVector::SphereVector(sackmesser_ros::Interface *interface, const std::string &name)
      : sackmesser_ros::Publisher<visualization_msgs::msg::MarkerArray, std::vector<gafro::Sphere<double>>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    SphereVector::~SphereVector() {}

    bool SphereVector::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame) &&            //
               server->loadParameter(ns + "color/r", &color_r, true) &&  //
               server->loadParameter(ns + "color/g", &color_g, true) &&  //
               server->loadParameter(ns + "color/b", &color_b, true) &&  //
               server->loadParameter(ns + "color/a", &color_a, true);
    }

    visualization_msgs::msg::MarkerArray SphereVector::createMessage(const std::vector<gafro::Sphere<double>> &spheres) const
    {
        visualization_msgs::msg::MarkerArray spheres_msg;

        int id = 0;

        for (const auto &sphere : spheres)
        {
            visualization_msgs::msg::Marker sphere_msg = convertToMarker(sphere);

            sphere_msg.id = ++id;

            sphere_msg.header.frame_id = config_.frame;
            sphere_msg.color.r = static_cast<float>(config_.color_r);
            sphere_msg.color.g = static_cast<float>(config_.color_g);
            sphere_msg.color.b = static_cast<float>(config_.color_b);
            sphere_msg.color.a = static_cast<float>(config_.color_a);

            spheres_msg.markers.push_back(sphere_msg);
        }

        return spheres_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::SphereVector, "gafro_sphere_vector");