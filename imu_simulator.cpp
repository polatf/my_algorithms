#include "imu_simulator.h"
#include <cstdlib>

IMUSimulator::IMUSimulator() : pitch(3.0), roll(0.0), yaw(-2.0), altitude(0.0) {
    thrust.left = 1000;
    thrust.right = 1000;
}

void IMUSimulator::setDisturbance(Disturbance d) {
    disturbance = d;
}

void IMUSimulator::setThrust(const MotorThrust& t) {
    thrust = t;
}

IMUData IMUSimulator::read() {
    pitch += 0.05;
    yaw += 0.02;

    // Asimetrik thrust nedeniyle oluşan roll momenti
    double moment = asymModel.computeRollingMoment(thrust);
    roll += moment * 0.0005;

    // Rüzgar sapması varsa yaw etkilenir
    if (disturbance.wind_drift) {
        yaw += disturbance.wind_force;
    }

    // Motor arızası varsa sol thrust sıfırlanır
    if (disturbance.motor_failure) {
        thrust.left = 0;
    }

    // İrtifa değişimi (sadeleştirilmiş model)
    double vertical_speed = pitch * 0.05;
    altitude += vertical_speed;

    // Gürültü ekleyerek sensör gerçekçiliği artır
    IMUData data;
    data.pitch = pitch + ((rand() % 100 - 50) / 100.0);
    data.roll = roll;
    data.yaw = yaw;
    data.pitch_rate = 0.2;
    data.roll_rate = 0.3;
    data.yaw_rate = 0.4;
    data.altitude = altitude;

    return data;
}
