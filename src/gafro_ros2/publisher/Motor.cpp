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
    {
        config_ = interface->getConfigurations()->load<Configuration>(ns);

        if (config_.publish_frame)
        {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(interface);
        }
    }

    MotorPublisher::~MotorPublisher() {}

    bool MotorPublisher::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "publish_frame", &publish_frame, true) &&  //
               server->loadParameter(ns + "frame_name", &frame_name);
    }

    geometry_msgs::msg::PoseStamped MotorPublisher::createMessage(const gafro::Motor<double> &motor) const
    {
        geometry_msgs::msg::PoseStamped pose_msg;

        pose_msg.header.frame_id = "world";
        pose_msg.header.stamp = getInterface()->now();
        pose_msg.pose = convertToPose(motor);

        if (config_.publish_frame)
        {
            geometry_msgs::msg::TransformStamped transform;

            transform.header.stamp = getInterface()->get_clock()->now();
            transform.header.frame_id = "world";
            transform.child_frame_id = config_.frame_name;

            transform.transform = convertToTransform(motor);

            tf_broadcaster_->sendTransform(transform);
        }

        return pose_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::MotorPublisher, "gafro_motor");