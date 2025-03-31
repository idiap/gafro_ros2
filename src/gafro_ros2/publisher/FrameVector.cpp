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

#include <gafro/gafro.hpp>
#include <gafro_ros2/conversion/Motor.hpp>
#include <gafro_ros2/conversion/Point.hpp>
//
#include <gafro_ros2/publisher/FrameVector.hpp>

namespace gafro_ros
{
    FrameVectorPublisher::FrameVectorPublisher(sackmesser_ros::Interface *interface, const std::string &ns)
      : sackmesser_ros::Publisher<visualization_msgs::msg::MarkerArray, std::vector<gafro::Motor<double>>>(interface, ns)
    {
        config_ = interface->getConfigurations()->load<Configuration>(ns);
    }

    FrameVectorPublisher::~FrameVectorPublisher() {}

    bool FrameVectorPublisher::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return server->loadParameter(ns + "color/r", &color.r, true) &&      //
               server->loadParameter(ns + "color/g", &color.g, true) &&      //
               server->loadParameter(ns + "color/b", &color.b, true) &&      //
               server->loadParameter(ns + "color/a", &color.a, true) &&      //
               server->loadParameter(ns + "use_color", &use_color, true) &&  //
               server->loadParameter(ns + "scale", &scale, true);
    }

    visualization_msgs::msg::MarkerArray FrameVectorPublisher::createMessage(const std::vector<gafro::Motor<double>> &motors) const
    {
        visualization_msgs::msg::MarkerArray path_msg;

        int id = 0;

        for (const auto &motor : motors)
        {
            visualization_msgs::msg::Marker r;

            r.header.frame_id = "world";
            // r.header.stamp = ros::Time();
            r.action = visualization_msgs::msg::Marker::ADD;
            r.ns = "x";
            r.id = id++;
            r.type = visualization_msgs::msg::Marker::ARROW;
            r.scale.x = 0.1f * config_.scale;
            r.scale.y = 0.2f * config_.scale;
            r.scale.z = 0.0f;
            if (!config_.use_color)
            {
                r.color.r = 1.0f;
                r.color.g = 0.0f;
                r.color.b = 0.0f;
                r.color.a = 1.0f;
            }
            else
            {
                r.color.r = static_cast<float>(config_.color.r);
                r.color.g = static_cast<float>(config_.color.g);
                r.color.b = static_cast<float>(config_.color.b);
                r.color.a = static_cast<float>(config_.color.a);
            }
            r.pose.position.x = 0.0;
            r.pose.position.y = 0.0;
            r.pose.position.z = 0.0;
            r.pose.orientation.w = 1.0;
            r.pose.orientation.x = 0.0;
            r.pose.orientation.y = 0.0;
            r.pose.orientation.z = 0.0;

            visualization_msgs::msg::Marker g;

            g.header.frame_id = "world";
            // g.header.stamp = ros::Time();
            g.action = visualization_msgs::msg::Marker::ADD;
            g.ns = "y";
            g.id = id++;
            g.type = visualization_msgs::msg::Marker::ARROW;
            g.scale.x = 0.1f * config_.scale;
            g.scale.y = 0.2f * config_.scale;
            g.scale.z = 0.0f;
            if (!config_.use_color)
            {
                g.color.r = 0.0f;
                g.color.g = 2.0f;
                g.color.b = 0.0f;
                g.color.a = 1.0f;
            }
            else
            {
                g.color.r = static_cast<float>(config_.color.r);
                g.color.g = static_cast<float>(config_.color.g);
                g.color.b = static_cast<float>(config_.color.b);
                g.color.a = static_cast<float>(config_.color.a);
            }
            g.pose.position.x = 0.0;
            g.pose.position.y = 0.0;
            g.pose.position.z = 0.0;
            g.pose.orientation.w = 1.0;
            g.pose.orientation.x = 0.0;
            g.pose.orientation.y = 0.0;
            g.pose.orientation.z = 0.0;

            visualization_msgs::msg::Marker b;

            b.header.frame_id = "world";
            // b.header.stamp = ros::Time();
            b.action = visualization_msgs::msg::Marker::ADD;
            b.ns = "z";
            b.id = id++;
            b.type = visualization_msgs::msg::Marker::ARROW;
            b.scale.x = 0.1f * config_.scale;
            b.scale.y = 0.2f * config_.scale;
            b.scale.z = 0.0f;
            if (!config_.use_color)
            {
                b.color.r = 0.0f;
                b.color.g = 0.0f;
                b.color.b = 1.0f;
                b.color.a = 1.0f;
            }
            else
            {
                b.color.r = static_cast<float>(config_.color.r);
                b.color.g = static_cast<float>(config_.color.g);
                b.color.b = static_cast<float>(config_.color.b);
                b.color.a = static_cast<float>(config_.color.a);
            }
            b.pose.position.x = 0.0;
            b.pose.position.y = 0.0;
            b.pose.position.z = 0.0;
            b.pose.orientation.w = 1.0;
            b.pose.orientation.x = 0.0;
            b.pose.orientation.y = 0.0;
            b.pose.orientation.z = 0.0;

            using Point = gafro::Point<double>;

            Point p1 = motor.apply(Point(0.0, 0.0, 0.0));
            Point p2 = motor.apply(Point(config_.scale, 0.0, 0.0));
            Point p3 = motor.apply(Point(0.0, config_.scale, 0.0));
            Point p4 = motor.apply(Point(0.0, 0.0, config_.scale));

            r.points.push_back(convertToPoint(p1));
            r.points.push_back(convertToPoint(p2));
            g.points.push_back(convertToPoint(p1));
            g.points.push_back(convertToPoint(p3));
            b.points.push_back(convertToPoint(p1));
            b.points.push_back(convertToPoint(p4));

            path_msg.markers.push_back(r);
            path_msg.markers.push_back(g);
            path_msg.markers.push_back(b);
        }

        return path_msg;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::FrameVectorPublisher, "gafro_frame_vector");