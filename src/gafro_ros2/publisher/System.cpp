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

#include <gafro/robot/System.hxx>
#include <gafro/robot/algorithm/ForwardKinematics.hpp>
#include <gafro_robot_descriptions/gafro_robot_descriptions_package_config.hpp>
#include <gafro_robot_descriptions/serialization/FilePath.hpp>
#include <gafro_robot_descriptions/serialization/SystemSerialization.hpp>
#include <gafro_robot_descriptions/serialization/Visual.hpp>
#include <gafro_ros2/conversion/Motor.hpp>
#include <gafro_ros2/publisher/System.hpp>
#include <sackmesser/UtilityFunctions.hpp>

namespace gafro_ros
{
    bool SystemPublisher::Configuration::load(const std::string &ns, const std::shared_ptr<sackmesser::Configurations> &server)
    {
        return                                                      // server->loadParameter(ns + "description", &description) &&  //
          server->loadParameter(ns + "frame", &frame) &&            //
          server->loadParameter(ns + "color/r", &color_r, true) &&  //
          server->loadParameter(ns + "color/g", &color_g, true) &&  //
          server->loadParameter(ns + "color/b", &color_b, true) &&  //
          server->loadParameter(ns + "color/a", &color_a, true);
    }

    SystemPublisher::SystemPublisher(sackmesser_ros::Interface *interface, const std::string &name)
      : sackmesser_ros::Publisher<visualization_msgs::msg::MarkerArray, gafro::System<double>, Eigen::MatrixXd, gafro::Motor<double>>(interface, name)
    {
        config_ = interface->getConfigurations()->load<Configuration>(name);
        // system_ = std::make_unique<gafro::System<double>>(gafro::SystemSerialization(gafro::FilePath(config_.description)).load());

        interface->addPublisher("gafro_motor_vector", sackmesser::splitString(name, '/')[1] + "_motors");
    }

    SystemPublisher::~SystemPublisher() = default;

    visualization_msgs::msg::Marker convertToMarker(const gafro::LinkVisual *visual)
    {
        visualization_msgs::msg::Marker marker;

        switch (visual->getType())
        {
        case gafro::LinkVisual::Type::SPHERE: {
            const gafro::visual::Sphere *sphere = static_cast<const gafro::visual::Sphere *>(visual);

            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = sphere->getRadius();
            marker.scale.y = sphere->getRadius();
            marker.scale.z = sphere->getRadius();
            marker.mesh_use_embedded_materials = false;

            break;
        }
        case gafro::LinkVisual::Type::MESH: {
            const gafro::visual::Mesh *mesh = static_cast<const gafro::visual::Mesh *>(visual);

            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            marker.mesh_resource = "file://" + mesh->getFilename();
            marker.scale.x = mesh->getScaleX();
            marker.scale.y = mesh->getScaleY();
            marker.scale.z = mesh->getScaleZ();
            marker.mesh_use_embedded_materials = true;

            break;
        }
        case gafro::LinkVisual::Type::CYLINDER: {
            const gafro::visual::Cylinder *cylinder = static_cast<const gafro::visual::Cylinder *>(visual);

            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.scale.x = 2.0 * cylinder->getRadius();
            marker.scale.y = 2.0 * cylinder->getRadius();
            marker.scale.z = cylinder->getLength();
            marker.mesh_use_embedded_materials = false;

            break;
        }
        case gafro::LinkVisual::Type::BOX: {
            const gafro::visual::Box *box = static_cast<const gafro::visual::Box *>(visual);

            marker.type = visualization_msgs::msg::Marker::CUBE;

            marker.scale.x = box->getDimX();
            marker.scale.y = box->getDimY();
            marker.scale.z = box->getDimZ();
            marker.mesh_use_embedded_materials = false;

            break;
        }
        }

        return marker;
    }

    visualization_msgs::msg::MarkerArray SystemPublisher::createMessage(const gafro::System<double> &system, const Eigen::MatrixXd &position,
                                                                        const gafro::Motor<double> &base) const
    {
        visualization_msgs::msg::MarkerArray all_markers;

        int id = 0;

        for (unsigned j = 0; j < position.cols(); ++j)
        {
            std::vector<gafro::Motor<double>> motors;

            gafro::ForwardKinematics<double> kinematics = system.computeForwardKinematics(position.col(j), base);

            visualization_msgs::msg::MarkerArray markers;

            for (const auto &pair : kinematics.getLinkPoses())
            {
                if (system.getLink(pair.first)->hasVisual())
                {
                    const gafro::LinkVisual *visual = system.getLink(pair.first)->getVisual();

                    visualization_msgs::msg::Marker visual_marker = convertToMarker(visual);

                    visual_marker.pose = convertToPose(pair.second * visual->getTransform());

                    visual_marker.header.frame_id = config_.frame;
                    visual_marker.ns = pair.first;
                    visual_marker.id = static_cast<int>(markers.markers.size());

                    markers.markers.push_back(visual_marker);
                }

                motors.push_back(pair.second);
            }

            for (visualization_msgs::msg::Marker &marker : markers.markers)
            {
                marker.color.r = static_cast<float>(config_.color_r);
                marker.color.g = static_cast<float>(config_.color_g);
                marker.color.b = static_cast<float>(config_.color_b);
                marker.color.a = static_cast<float>(config_.color_a);

                marker.mesh_use_embedded_materials = true;

                marker.id = id + marker.id;

                all_markers.markers.push_back(marker);
            }

            id += static_cast<int>(position.rows());

            getInterface()->getCallbacks()->invoke("robot_motors", motors);
        }

        return all_markers;
    }

}  // namespace gafro_ros

REGISTER_CLASS(sackmesser_ros::base::Publisher, gafro_ros::SystemPublisher, "gafro_system")