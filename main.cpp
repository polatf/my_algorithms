#include <iostream>
#include <iomanip>
#include "imu_simulator.h"
#include "kalman_filter.h"
#include "pid_controller.h"
#include "roc_evaluator.h"
#include "fsm_controller.h"

int main() {

    IMUSimulator imuSim;
    FSMController fsm;
    ROCEvaluator rocEval;

    // PID Controller
    PID pitchPID(2.0, 0.4, 0.9);
    PID rollPID(2.5, 0.3, 1.2);
    PID yawPID(1.8, 0.5, 0.7);

    // Kalman filtreleri 
    KalmanFilter kalmanPitch, kalmanRoll, kalmanYaw;

    // ————— SİMULASYON PARAMETRELERİ ————— //
    double dt = 0.05;             // zaman adımı (saniye cinsinden)
    double pitch_target = 10.0;  // istenen burun açısı
    double roll_target = 0.0;
    double yaw_target = 0.0;

    // ————— BOZULMA DURUMU (hata senaryosu) ————— //
    Disturbance d;
    d.wind_drift = false;
    d.wind_force = 0.03;
    d.motor_failure = true;
    imuSim.setDisturbance(d);

    // ————— MOTOR İTİŞ (normalde simetrik) ————— //
    MotorThrust thrust;
    thrust.left = 1000;
    thrust.right = 1000;
    imuSim.setThrust(thrust);

    // ————— SİMÜLASYON DÖNGÜSÜ ————— //
    for (int i = 0; i < 150; ++i) {
        IMUData imu_data = imuSim.read();  // IMU verisini oku

        // ROC hesapla
        double roc = rocEval.computeROC(imu_data.altitude, dt);

        // Hataları değerlendir
        bool wind_sapmasi_var = std::abs(imu_data.yaw) > 10.0;
        bool motor_arizasi_var = d.motor_failure;

        // FSM güncelle
        fsm.updateState(imu_data, motor_arizasi_var, wind_sapmasi_var, roc);

        // Eğer FSM ACIL_DONUS modundaysa PID kontrol yapma
        if (fsm.getState() != HAZIRLIK && fsm.getState() != ACIL_DONUS) {
            // Kalman filtre ile açıyı iyileştir
            double pitch_filtered = kalmanPitch.getAngle(imu_data.pitch, imu_data.pitch_rate, dt);
            double roll_filtered  = kalmanRoll.getAngle(imu_data.roll, imu_data.roll_rate, dt);
            double yaw_filtered   = kalmanYaw.getAngle(imu_data.yaw, imu_data.yaw_rate, dt);

            // PID hesapla
            double elevator = pitchPID.calculate(pitch_target, pitch_filtered, dt);
            double aileron  = rollPID.calculate(roll_target, roll_filtered, dt);
            double rudder   = yawPID.calculate(yaw_target, yaw_filtered, dt);

            // Komut çıktısı
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "[KOMUT] Elevator: " << elevator
                      << " | Aileron: " << aileron
                      << " | Rudder: " << rudder << std::endl;
        }

        // Uçuş verilerini her adımda yazdır
        std::cout << "Zaman: " << std::setw(6) << i * dt << " s"
                  << " | Pitch: " << imu_data.pitch
                  << " | Roll: " << imu_data.roll
                  << " | Yaw: " << imu_data.yaw
                  << " | Altitude: " << imu_data.altitude
                  << " | ROC: " << roc
                  << " | FSM: " << fsm.getState() << "\n\n";
    }

    return 0;
}
