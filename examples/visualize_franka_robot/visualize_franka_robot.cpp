#include <gafro/gafro.hpp>
#include <gafro_robot_descriptions/FrankaEmikaRobot.hpp>
#include <sackmesser/Callbacks.hpp>
#include <sackmesser_ros2/Interface.hpp>

int main(int argc, char **argv)
{
    auto interface = sackmesser_ros::Interface::create(argc, argv, "visualize_franka_robot", "gafro_ros2");

    gafro::FrankaEmikaRobot<double> panda;

    interface->loop([&]() {
        Eigen::MatrixXd q = panda.getRandomConfiguration();

        gafro::Motor<double> ee_motor = panda.getEEMotor(q);
        gafro::Point<double> ee_point = ee_motor.apply(gafro::Point<double>());
        gafro::Line<double> ee_line = ee_motor.apply(gafro::Line<double>::Z());

        interface->getCallbacks()->invoke("robot", panda.getSystem(), q, gafro::Motor<double>());
        interface->getCallbacks()->invoke("ee_motor", ee_motor);
        interface->getCallbacks()->invoke("ee_point", ee_point);
        interface->getCallbacks()->invoke("ee_line", ee_line);
    });

    return 1;
}