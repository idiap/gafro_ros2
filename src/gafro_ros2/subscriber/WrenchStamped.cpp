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

#include <gafro/physics/Wrench.hxx>
//
#include <gafro_ros2/subscriber/WrenchStamped.hpp>

namespace gafro_ros
{
    SubscriberWrench::SubscriberWrench(sackmesser_ros::Interface *interface, const std::string &topic, const std::string &callback_queue)  //
      : sackmesser_ros::Subscriber<geometry_msgs::msg::WrenchStamped, gafro::Wrench<double>>(interface, topic, callback_queue)
    {}

    gafro::Wrench<double> SubscriberWrench::convert(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench_msg) const
    {
        gafro::Wrench<double> wrench;

        wrench.multivector().set<gafro::blades::e01>(wrench_msg->wrench.force.x);
        wrench.multivector().set<gafro::blades::e02>(wrench_msg->wrench.force.y);
        wrench.multivector().set<gafro::blades::e03>(wrench_msg->wrench.force.z);

        wrench.multivector().set<gafro::blades::e23>(wrench_msg->wrench.torque.x);
        wrench.multivector().set<gafro::blades::e13>(-wrench_msg->wrench.torque.y);
        wrench.multivector().set<gafro::blades::e12>(wrench_msg->wrench.torque.z);

        return wrench;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Subscriber, gafro_ros::SubscriberWrench, "gafro_wrench");