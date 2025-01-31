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

#pragma once

#include <gafro/algebra/cga/Motor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//
#include <sackmesser_ros2/Publisher.hpp>

namespace gafro_ros
{
    class MotorPublisher : public sackmesser_ros::Publisher<geometry_msgs::msg::PoseStamped, gafro::Motor<double>>
    {
      public:
        MotorPublisher(sackmesser_ros::Interface *interface, const std::string &ns);

        ~MotorPublisher();

        geometry_msgs::msg::PoseStamped createMessage(const gafro::Motor<double> &motor) const;
    };

}  // namespace gafro_ros