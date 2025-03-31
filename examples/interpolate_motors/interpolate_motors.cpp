#include <gafro/gafro.hpp>
#include <sackmesser_ros2/Interface.hpp>

int main(int argc, char **argv)
{
    auto interface = sackmesser_ros::Interface::create(argc, argv, "interpolate_motors", "gafro_ros2");

    // double t0 = 0.0;
    // double t1 = 0.5;
    // double t2 = 1.0;

    // gafro::Motor<double> m0 = gafro::Motor<double>::Random();
    // gafro::Motor<double> m1 = gafro::Motor<double>::Random();
    // gafro::Motor<double> m2 = gafro::Motor<double>::Random();

    // gafro::Motor<double>::Generator b0 = m0.log();
    // gafro::Motor<double>::Generator b1 = m1.log();
    // gafro::Motor<double>::Generator b2 = m2.log();

    // std::vector<gafro::Motor<double>> key_motors = { m0, m1, m2 };
    // std::vector<gafro::Motor<double>> motors;

    // for (double t = t0; t <= t2; t += 0.01)
    // {
    //     double w0 = (t - t1) * (t - t2) / ((t0 - t1) * (t0 - t2));
    //     double w1 = (t - t0) * (t - t2) / ((t1 - t0) * (t1 - t2));
    //     double w2 = (t - t0) * (t - t1) / ((t2 - t0) * (t2 - t1));

    //     gafro::Motor<double>::Generator b = w0 * b0 + w1 * b1 + w2 * b2;
    //     gafro::Motor<double> m = gafro::Motor<double>::exp(b);
    //     motors.push_back(m);
    // }

    gafro::Motor<double> m0 = gafro::Motor<double>::Random();
    gafro::Motor<double> m1 = gafro::Motor<double>::Random();

    gafro::Motor<double> mi = m1 * m0.reverse();
    gafro::Motor<double>::Generator bi = mi.log();

    std::vector<gafro::Motor<double>> key_motors = { m0, m1 };
    std::vector<gafro::Motor<double>> motors;

    for (double t = 0.0; t <= 1.0; t += 0.1)
    {
        gafro::Motor<double>::Generator b = t * bi;
        gafro::Motor<double> m = gafro::Motor<double>::exp(b) * m0;
        motors.push_back(m);
    }

    interface->loop([&]() {
        interface->getCallbacks()->invoke("motors", motors);
        interface->getCallbacks()->invoke("key_motors", key_motors);
    });

    return 0;
}