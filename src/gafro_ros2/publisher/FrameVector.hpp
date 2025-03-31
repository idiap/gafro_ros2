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
#include <visualization_msgs/msg/marker_array.hpp>
//
#include <sackmesser_ros2/Publisher.hpp>

namespace gafro_ros
{
    class FrameVectorPublisher : public sackmesser_ros::Publisher<visualization_msgs::msg::MarkerArray, std::vector<gafro::Motor<double>>>
    {
      public:
        FrameVectorPublisher(sackmesser_ros::Interface *interface, const std::string &ns);

        ~FrameVectorPublisher();

        visualization_msgs::msg::MarkerArray createMessage(const std::vector<gafro::Motor<double>> &motors) const;

      private:
        struct Configuration : public sackmesser::Configuration
        {
            bool load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server);

            struct Color
            {
                double r;
                double g;
                double b;
                double a;
            } color;

            double scale;

            bool use_color;
        };

        Configuration config_;
    };

}  // namespace gafro_ros