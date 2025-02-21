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
#include <gafro/physics/Wrench.hxx>
//
#include <gafro_ros2/conversion/Wrench.hpp>
//
#include <gafro_ros2/publisher/Wrench.hpp>

namespace gafro_ros
{
    Wrench::Wrench(sackmesser_ros::Interface *interface, const std::string &name)
      : sackmesser_ros::Publisher<geometry_msgs::msg::WrenchStamped, gafro::Wrench<double>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
    }

    Wrench::~Wrench() {}

    bool Wrench::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "frame", &frame);
    }

    geometry_msgs::msg::WrenchStamped Wrench::createMessage(const gafro::Wrench<double> &wrench) const
    {
        geometry_msgs::msg::WrenchStamped wrench_msg;

        wrench_msg.header.frame_id = config_.frame;

        wrench_msg.wrench = convert(wrench);

        return wrench_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::Wrench, "gafro_wrench");