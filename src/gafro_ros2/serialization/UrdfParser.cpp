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

#include <fstream>
//
#include <gafro_ros2/serialization/UrdfParser.hpp>

namespace gafro_ros
{

    UrdfParser::UrdfParser(const std::string &filename)
    {
        model_.initFile(filename);
    }

    UrdfParser::~UrdfParser() = default;

    YAML::Node UrdfParser::parseToYaml() const
    {
        YAML::Node node;

        std::vector<urdf::LinkSharedPtr> links;
        model_.getLinks(links);

        urdf::LinkConstSharedPtr link = model_.getRoot();

        node["system"]["name"] = model_.getName();

        if (link->name == "world")
        {
            node["system"]["root_link"] = link->child_links.front()->name;
        }
        else
        {
            node["system"]["root_link"] = link->name;
        }

        for (const urdf::LinkSharedPtr &l : links)
        {
            if (l->name == "world")
            {
                continue;
            }

            for (const urdf::JointSharedPtr &j : l->child_joints)
            {
                switch (j->type)
                {
                case urdf::Joint::REVOLUTE: {
                    node["joints"][j->name]["type"] = "revolute";
                    break;
                }
                case urdf::Joint::PRISMATIC: {
                    node["joints"][j->name]["type"] = "prismatic";
                    break;
                }
                case urdf::Joint::FIXED: {
                    node["joints"][j->name]["type"] = "fixed";
                    break;
                }
                default: {
                    node["joints"][j->name]["type"] = "unknown";
                    continue;
                }
                }

                node["joints"][j->name]["name"] = j->name;
                node["joints"][j->name]["parent_link"] = j->parent_link_name;
                node["joints"][j->name]["child_link"] = j->child_link_name;

                node["joints"][j->name]["transform"]["translation"]["x"] = j->parent_to_joint_origin_transform.position.x;
                node["joints"][j->name]["transform"]["translation"]["y"] = j->parent_to_joint_origin_transform.position.y;
                node["joints"][j->name]["transform"]["translation"]["z"] = j->parent_to_joint_origin_transform.position.z;
                node["joints"][j->name]["transform"]["rotation"]["w"] = j->parent_to_joint_origin_transform.rotation.w;
                node["joints"][j->name]["transform"]["rotation"]["x"] = j->parent_to_joint_origin_transform.rotation.x;
                node["joints"][j->name]["transform"]["rotation"]["y"] = j->parent_to_joint_origin_transform.rotation.y;
                node["joints"][j->name]["transform"]["rotation"]["z"] = j->parent_to_joint_origin_transform.rotation.z;
                node["joints"][j->name]["axis"]["x"] = j->axis.x;
                node["joints"][j->name]["axis"]["y"] = j->axis.y;
                node["joints"][j->name]["axis"]["z"] = j->axis.z;

                if (j->limits)
                {
                    node["joints"][j->name]["limits"]["lower"] = j->limits->lower;
                    node["joints"][j->name]["limits"]["upper"] = j->limits->upper;
                    node["joints"][j->name]["limits"]["velocity"] = j->limits->velocity;
                    node["joints"][j->name]["limits"]["effort"] = j->limits->effort;
                }

                node["links"][l->name]["joints"].push_back(j->name);
            }

            node["links"][l->name]["name"] = l->name;
            if (l->parent_joint && l->parent_joint->name != "world_joint")
            {
                node["links"][l->name]["parent_joint"] = l->parent_joint->name;
            }

            if (l->inertial)
            {
                node["links"][l->name]["inertial"]["mass"] = l->inertial->mass;
                node["links"][l->name]["inertial"]["inertia"]["xx"] = l->inertial->ixx;
                node["links"][l->name]["inertial"]["inertia"]["xy"] = l->inertial->ixy;
                node["links"][l->name]["inertial"]["inertia"]["yy"] = l->inertial->iyy;
                node["links"][l->name]["inertial"]["inertia"]["yz"] = l->inertial->iyz;
                node["links"][l->name]["inertial"]["inertia"]["zz"] = l->inertial->izz;
                node["links"][l->name]["inertial"]["inertia"]["xz"] = l->inertial->ixz;
                node["links"][l->name]["inertial"]["origin"]["position"]["x"] = l->inertial->origin.position.x;
                node["links"][l->name]["inertial"]["origin"]["position"]["y"] = l->inertial->origin.position.y;
                node["links"][l->name]["inertial"]["origin"]["position"]["z"] = l->inertial->origin.position.z;
                node["links"][l->name]["inertial"]["origin"]["orientation"]["w"] = l->inertial->origin.rotation.w;
                node["links"][l->name]["inertial"]["origin"]["orientation"]["x"] = l->inertial->origin.rotation.x;
                node["links"][l->name]["inertial"]["origin"]["orientation"]["y"] = l->inertial->origin.rotation.y;
                node["links"][l->name]["inertial"]["origin"]["orientation"]["z"] = l->inertial->origin.rotation.z;
            }
            else
            {
                node["links"][l->name]["inertial"]["mass"] = 0.0;
                node["links"][l->name]["inertial"]["inertia"]["xx"] = 0.0;
                node["links"][l->name]["inertial"]["inertia"]["xy"] = 0.0;
                node["links"][l->name]["inertial"]["inertia"]["yy"] = 0.0;
                node["links"][l->name]["inertial"]["inertia"]["yz"] = 0.0;
                node["links"][l->name]["inertial"]["inertia"]["zz"] = 0.0;
                node["links"][l->name]["inertial"]["inertia"]["xz"] = 0.0;
                node["links"][l->name]["inertial"]["origin"]["position"]["x"] = 0.0;
                node["links"][l->name]["inertial"]["origin"]["position"]["y"] = 0.0;
                node["links"][l->name]["inertial"]["origin"]["position"]["z"] = 0.0;
                node["links"][l->name]["inertial"]["origin"]["orientation"]["w"] = 1.0;
                node["links"][l->name]["inertial"]["origin"]["orientation"]["x"] = 0.0;
                node["links"][l->name]["inertial"]["origin"]["orientation"]["y"] = 0.0;
                node["links"][l->name]["inertial"]["origin"]["orientation"]["z"] = 0.0;
            }

            if (l->visual)
            {
                node["links"][l->name]["visual"]["origin"]["position"]["x"] = l->visual->origin.position.x;
                node["links"][l->name]["visual"]["origin"]["position"]["y"] = l->visual->origin.position.y;
                node["links"][l->name]["visual"]["origin"]["position"]["z"] = l->visual->origin.position.z;
                node["links"][l->name]["visual"]["origin"]["orientation"]["w"] = l->visual->origin.rotation.w;
                node["links"][l->name]["visual"]["origin"]["orientation"]["x"] = l->visual->origin.rotation.x;
                node["links"][l->name]["visual"]["origin"]["orientation"]["y"] = l->visual->origin.rotation.y;
                node["links"][l->name]["visual"]["origin"]["orientation"]["z"] = l->visual->origin.rotation.z;

                switch (l->visual->geometry->type)
                {
                case urdf::Geometry::SPHERE: {
                    node["links"][l->name]["visual"]["type"] = "sphere";

                    urdf::Sphere *sphere = static_cast<urdf::Sphere *>(l->visual->geometry.get());

                    node["links"][l->name]["visual"]["radius"] = sphere->radius;

                    break;
                }
                case urdf::Geometry::MESH: {
                    node["links"][l->name]["visual"]["type"] = "mesh";

                    urdf::Mesh *mesh = static_cast<urdf::Mesh *>(l->visual->geometry.get());

                    node["links"][l->name]["visual"]["filename"] = mesh->filename;
                    node["links"][l->name]["visual"]["scale"]["x"] = mesh->scale.x;
                    node["links"][l->name]["visual"]["scale"]["y"] = mesh->scale.y;
                    node["links"][l->name]["visual"]["scale"]["z"] = mesh->scale.z;

                    break;
                }
                case urdf::Geometry::CYLINDER: {
                    node["links"][l->name]["visual"]["type"] = "cylinder";

                    urdf::Cylinder *cylinder = static_cast<urdf::Cylinder *>(l->visual->geometry.get());

                    node["links"][l->name]["visual"]["length"] = cylinder->length;
                    node["links"][l->name]["visual"]["radius"] = cylinder->radius;

                    break;
                }
                case urdf::Geometry::BOX: {
                    node["links"][l->name]["visual"]["type"] = "box";

                    urdf::Box *box = static_cast<urdf::Box *>(l->visual->geometry.get());

                    node["links"][l->name]["visual"]["length"] = box->dim.x;
                    node["links"][l->name]["visual"]["width"] = box->dim.y;
                    node["links"][l->name]["visual"]["height"] = box->dim.z;

                    break;
                }
                }
            }
        }

        return node;
    }

    void UrdfParser::writeToYaml(const std::string &filename) const
    {
        YAML::Emitter emitter;
        emitter << parseToYaml();

        std::ofstream file(filename.c_str());
        file << emitter.c_str();
    }

}  // namespace gafro_ros