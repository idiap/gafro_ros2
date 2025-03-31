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
#include <gafro_ros2/conversion/Motor.hpp>

namespace gafro_ros
{

    geometry_msgs::msg::Pose convertToPose(const gafro::Motor<double> &motor)
    {
        geometry_msgs::msg::Pose pose;

        Eigen::Quaterniond quaternion = motor.getRotor().quaternion();
        Eigen::Vector3d translation = motor.getTranslator().toTranslationVector();

        pose.position.x = translation.x();
        pose.position.y = translation.y();
        pose.position.z = translation.z();

        pose.orientation.w = quaternion.w();
        pose.orientation.x = quaternion.x();
        pose.orientation.y = quaternion.y();
        pose.orientation.z = quaternion.z();

        return pose;
    }

    gafro::Motor<double> convertFromPose(const geometry_msgs::msg::Pose &pose)
    {
        Eigen::Quaterniond quaternion({ pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z });
        Eigen::Vector3d translation({ pose.position.x, pose.position.y, pose.position.z });

        return gafro::Motor<double>(translation, quaternion);
    }

    geometry_msgs::msg::Transform convertToTransform(const gafro::Motor<double> &motor)
    {
        geometry_msgs::msg::Transform transform;

        Eigen::Vector<double, 3> translation = motor.getTranslator().toTranslationVector();
        Eigen::Quaternion<double> rotation = motor.getRotor().quaternion();

        transform.translation.x = translation.x();
        transform.translation.y = translation.y();
        transform.translation.z = translation.z();
        transform.rotation.x = rotation.x();
        transform.rotation.y = rotation.y();
        transform.rotation.z = rotation.z();
        transform.rotation.w = rotation.w();

        return transform;
    }

    // tf::Transform convertToFrame(const gafro::Motor<double> &motor)
    // {
    //     tf::Transform transform;

    //     gafro::Rotor<double> rotor = motor.getRotor();
    //     gafro::Point<double> point = motor.apply(gafro::Point<double>());

    //     transform.setOrigin(tf::Vector3(point.get<gafro::blades::e1>(), point.get<gafro::blades::e2>(), point.get<gafro::blades::e3>()));
    //     tf::Quaternion q;
    //     q.setW(rotor.quaternion().w());
    //     q.setX(rotor.quaternion().x());
    //     q.setY(rotor.quaternion().y());
    //     q.setZ(rotor.quaternion().z());
    //     transform.setRotation(q);

    //     return transform;
    // }

}  // namespace gafro_ros