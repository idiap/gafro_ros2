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

    // visualization_msgs::msg::MarkerArray convertToMarkerArray(const gafro::Motor<double> &motor, const std::string &frame, const int &base_id,
    //                                                           const double &scale, const double &opacity)
    // {
    //     int id = base_id;

    //     visualization_msgs::msg::Marker r;

    //     r.header.frame_id = frame;
    //     // r.header.stamp = ros::Time();
    //     r.action = visualization_msgs::msg::Marker::ADD;
    //     r.ns = "x";
    //     r.id = id++;
    //     r.type = visualization_msgs::msg::Marker::ARROW;
    //     r.scale.x = 0.001f;
    //     r.scale.y = 0.002f;
    //     r.scale.z = 0.0f;
    //     r.color.r = 1.0f;
    //     r.color.g = 0.0f;
    //     r.color.b = 0.0f;
    //     r.color.a = static_cast<float>(opacity);
    //     r.pose.position.x = 0.0;
    //     r.pose.position.y = 0.0;
    //     r.pose.position.z = 0.0;
    //     r.pose.orientation.w = 1.0;
    //     r.pose.orientation.x = 0.0;
    //     r.pose.orientation.y = 0.0;
    //     r.pose.orientation.z = 0.0;

    //     visualization_msgs::msg::Marker g;

    //     g.header.frame_id = frame;
    //     // g.header.stamp = ros::Time();
    //     g.action = visualization_msgs::msg::Marker::ADD;
    //     g.ns = "y";
    //     g.id = id++;
    //     g.type = visualization_msgs::msg::Marker::ARROW;
    //     g.scale.x = 0.001f;
    //     g.scale.y = 0.002f;
    //     g.scale.z = 0.0f;
    //     g.color.r = 0.0f;
    //     g.color.g = 2.0f;
    //     g.color.b = 0.0f;
    //     g.color.a = static_cast<float>(opacity);
    //     g.pose.position.x = 0.0;
    //     g.pose.position.y = 0.0;
    //     g.pose.position.z = 0.0;
    //     g.pose.orientation.w = 1.0;
    //     g.pose.orientation.x = 0.0;
    //     g.pose.orientation.y = 0.0;
    //     g.pose.orientation.z = 0.0;

    //     visualization_msgs::msg::Marker b;

    //     b.header.frame_id = frame;
    //     // b.header.stamp = ros::Time();
    //     b.action = visualization_msgs::msg::Marker::ADD;
    //     b.ns = "z";
    //     b.id = id++;
    //     b.type = visualization_msgs::msg::Marker::ARROW;
    //     b.scale.x = 0.001f;
    //     b.scale.y = 0.002f;
    //     b.scale.z = 0.0f;
    //     b.color.r = 0.0f;
    //     b.color.g = 0.0f;
    //     b.color.b = 1.0f;
    //     b.color.a = static_cast<float>(opacity);
    //     b.pose.position.x = 0.0;
    //     b.pose.position.y = 0.0;
    //     b.pose.position.z = 0.0;
    //     b.pose.orientation.w = 1.0;
    //     b.pose.orientation.x = 0.0;
    //     b.pose.orientation.y = 0.0;
    //     b.pose.orientation.z = 0.0;

    //     using Point = gafro::Point<double>;

    //     Point p1 = motor.apply(Point(0.0, 0.0, 0.0));
    //     Point p2 = motor.apply(Point(scale, 0.0, 0.0));
    //     Point p3 = motor.apply(Point(0.0, scale, 0.0));
    //     Point p4 = motor.apply(Point(0.0, 0.0, scale));

    //     // r.points.push_back(convertToPointMsg(p1));
    //     // r.points.push_back(convertToPointMsg(p2));
    //     // g.points.push_back(convertToPointMsg(p1));
    //     // g.points.push_back(convertToPointMsg(p3));
    //     // b.points.push_back(convertToPointMsg(p1));
    //     // b.points.push_back(convertToPointMsg(p4));

    //     visualization_msgs::msg::MarkerArray marker_array;

    //     marker_array.markers.push_back(r);
    //     marker_array.markers.push_back(g);
    //     marker_array.markers.push_back(b);

    //     return marker_array;
    // }

}  // namespace gafro_ros