#include <iostream>
#include <string>
//
#include <gafro_ros2/serialization/UrdfParser.hpp>

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "usage: convert_urdf input.urdf output.yaml" << std::endl;

        return -1;
    }

    std::string input = argv[1];
    std::string output = argv[2];

    gafro_ros::UrdfParser urdf_parser(input);

    urdf_parser.writeToYaml(output);

    return 0;
}
