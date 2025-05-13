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

#include <gafro/physics/Twist.hxx>
//
#include <gafro_ros2/subscriber/TwistStamped.hpp>

namespace gafro_ros
{
    SubscriberTwist::SubscriberTwist(sackmesser_ros::Interface *interface, const std::string &topic, const std::string &callback_queue)  //
      : sackmesser_ros::Subscriber<geometry_msgs::msg::TwistStamped, gafro::Twist<double>>(interface, topic, callback_queue)
    {}

    gafro::Twist<double> SubscriberTwist::convert(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg) const
    {
        gafro::Twist<double> twist;

        twist.multivector().set<gafro::blades::e1i>(twist_msg->twist.linear.x);
        twist.multivector().set<gafro::blades::e2i>(twist_msg->twist.linear.y);
        twist.multivector().set<gafro::blades::e3i>(twist_msg->twist.linear.z);

        twist.multivector().set<gafro::blades::e23>(twist_msg->twist.angular.x);
        twist.multivector().set<gafro::blades::e13>(-twist_msg->twist.angular.y);
        twist.multivector().set<gafro::blades::e12>(twist_msg->twist.angular.z);

        return twist;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Subscriber, gafro_ros::SubscriberTwist, "gafro_twist");