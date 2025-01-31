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

#include <geometry_msgs/msg/wrench_stamped.hpp>
//
#include <sackmesser_ros2/Subscriber.hpp>

namespace gafro
{
    template <class T>
    class Wrench;
}

namespace gafro_ros
{
    class SubscriberWrench : public sackmesser_ros::Subscriber<geometry_msgs::msg::WrenchStamped, gafro::Wrench<double>>
    {
      public:
        SubscriberWrench(sackmesser_ros::Interface *interface, const std::string &topic, const std::string &callback_queue);

      private:
        gafro::Wrench<double> convert(const geometry_msgs::msg::WrenchStamped::SharedPtr message) const;
    };
}  // namespace gafro_ros