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

#include <yaml-cpp/yaml.h>

#include <string>
#include <urdf/model.hpp>

namespace gafro_ros
{

    class UrdfParser
    {
      public:
        UrdfParser(const std::string &filename);

        virtual ~UrdfParser();

        YAML::Node parseToYaml() const;

        void writeToYaml(const std::string &filename) const;

      protected:
      private:
        urdf::Model model_;
    };

}  // namespace gafro_ros