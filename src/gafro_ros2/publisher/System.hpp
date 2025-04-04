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

#include <Eigen/Geometry>
#include <sackmesser_ros2/Publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace gafro
{
    template <class T>
    class Motor;

    template <class T>
    class System;
}  // namespace gafro

namespace gafro_ros
{
    class SystemPublisher
      : public sackmesser_ros::Publisher<visualization_msgs::msg::MarkerArray, gafro::System<double>, Eigen::MatrixXd, gafro::Motor<double>>
    {
      public:
        SystemPublisher(sackmesser_ros::Interface *interface, const std::string &name);

        ~SystemPublisher();

        struct Configuration : public sackmesser::Configuration
        {
            bool load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server);

            std::string frame;

            double color_r = 1.0;
            double color_g = 1.0;
            double color_b = 1.0;
            double color_a = 1.0;
        };

        visualization_msgs::msg::MarkerArray createMessage(const gafro::System<double> &system, const Eigen::MatrixXd &position,
                                                           const gafro::Motor<double> &base) const;

      protected:
      private:
        Configuration config_;
    };

}  // namespace gafro_ros