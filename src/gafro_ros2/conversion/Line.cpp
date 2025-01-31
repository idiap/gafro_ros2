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
#include <gafro_ros2/conversion/Line.hpp>

namespace gafro_ros
{

    visualization_msgs::msg::Marker convertToMarker(const gafro::Line<double> &line)
    {
        visualization_msgs::msg::Marker line_msg;

        auto expr = gafro::Scalar<double>(-1.0) * (line | gafro::E0<double>(1.0)).evaluate() + gafro::E0i<double>(0.5 * line.norm());

        gafro::PointPair<double> pp;

        pp.set<3>(expr.get<3>());
        pp.set<5>(expr.get<5>());
        pp.set<6>(expr.get<6>());
        pp.set<9>(expr.get<9>());
        pp.set<10>(expr.get<10>());
        pp.set<12>(expr.get<12>());
        pp.set<17>(expr.get<17>());

        Eigen::Vector3d p1 = pp.getPoint1().vector().middleRows(1, 3);
        Eigen::Vector3d p2 = pp.getPoint2().vector().middleRows(1, 3);

        Eigen::Vector3d direction = (p2 - p1).normalized();

        if (direction.norm() > 1e-8)
        {
            line_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;

            geometry_msgs::msg::Point p_msg_1;
            p_msg_1.x = p1.x() + 100.0 * direction.x();
            p_msg_1.y = p1.y() + 100.0 * direction.y();
            p_msg_1.z = p1.z() + 100.0 * direction.z();
            line_msg.points.push_back(p_msg_1);

            geometry_msgs::msg::Point p_msg_2;
            p_msg_2.x = p1.x() - 100.0 * direction.x();
            p_msg_2.y = p1.y() - 100.0 * direction.y();
            p_msg_2.z = p1.z() - 100.0 * direction.z();
            line_msg.points.push_back(p_msg_2);
        }
        else if (!direction.hasNaN())
        {
            line_msg.type = visualization_msgs::msg::Marker::POINTS;

            geometry_msgs::msg::Point p_msg;

            p1 = p1.normalized();

            p_msg.x = p1.x();
            p_msg.y = p1.y();
            p_msg.z = p1.z();
            line_msg.points.push_back(p_msg);
        }

        return line_msg;
    }

}  // namespace gafro_ros