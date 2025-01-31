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

#include <gafro_ros2/conversion/Motor.hpp>
//
#include <gafro_ros2/publisher/Motor.hpp>

namespace gafro_ros
{
    MotorPublisher::MotorPublisher(sackmesser_ros::Interface *interface, const std::string &ns)
      : sackmesser_ros::Publisher<geometry_msgs::msg::PoseStamped, gafro::Motor<double>>(interface, ns)
    {}

    MotorPublisher::~MotorPublisher() {}

    geometry_msgs::msg::PoseStamped MotorPublisher::createMessage(const gafro::Motor<double> &motor) const
    {
        geometry_msgs::msg::PoseStamped pose_msg;

        pose_msg.header.frame_id = "world";
        pose_msg.header.stamp = getInterface()->now();
        pose_msg.pose = convertToPose(motor);

        return pose_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::MotorPublisher, "gafro_motor");