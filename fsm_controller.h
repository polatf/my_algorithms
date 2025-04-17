#ifndef FSM_CONTROLLER_H
#define FSM_CONTROLLER_H

#include <iostream>
#include <cmath>
#include "imu_simulator.h"

// Uçuş durumlarını 
enum FlightState {
    HAZIRLIK,
    KALKIS,
    TIRMANIS,
    NOMINAL_FLIGHT,
    MOTOR_FAILURE,
    WIND_CORRECTION,
    ACIL_DONUS
};

// Uçuş yönetim birimi
class FSMController {
private:
    FlightState currentState;  // Şu anki uçuş durumu

public:
    FSMController() : currentState(HAZIRLIK) {}

    // FSM'nin güncellenmesi – her döngüde çağrılır
    void updateState(const IMUData& imu, bool motorFail, bool windDrift, double roc) {
        switch (currentState) {
            case HAZIRLIK:
                std::cout << " HAZIRLIK tamam. KALKISa geçiliyor.\n";
                currentState = KALKIS;
                break;

            case KALKIS:
                // Kalkış sırasında burun açısı istenen değeri geçerse tırmanışa geç
                if (imu.pitch > 8.0) {
                    std::cout << " ROTASYON! TIRMANIŞ moduna geçildi.\n";
                    currentState = TIRMANIS;
                }
                break;

            case TIRMANIS:
                if (motorFail && roc < 2.4) {
                    std::cout << "  ROC çok düşük. Kalkis iptal ediliyor. ACIL_DONUS moduna geçildi.\n";
                    currentState = ACIL_DONUS;
                } else if (motorFail) {
                    std::cout << " MOTOR ARIZASI. MOTOR_FAILURE moduna geçildi.\n";
                    currentState = MOTOR_FAILURE;
                } else if (windDrift) {
                    std::cout << " RÜZGAR SAPMASI tespit edildi. WIND_CORRECTION aktif.\n";
                    currentState = WIND_CORRECTION;
                } else {
                    currentState = NOMINAL_FLIGHT;
                }
                break;

            case MOTOR_FAILURE:
                std::cout << " Motor arizasi yönetiliyor. PID roll düzeltmesi devam ediyor.\n";
                break;

            case WIND_CORRECTION:
                std::cout << " Rüzgar etkisi bastırılıyor. PID yaw kontrolü devrede.\n";
                break;

            case NOMINAL_FLIGHT:
                std::cout << " Düzgün uçuş devam ediyor.\n";
                break;

            case ACIL_DONUS:
                std::cout << "ACİL DÖNÜŞ başlatıldı! Kontrol yer istasyonuna geçebilir.\n";
                break;
        }
    }

    // FSM'deki mevcut durumu getir
    FlightState getState() const {
        return currentState;
    }

    // Gerekirse sıfırla (test için)
    void reset() {
        currentState = HAZIRLIK;
    }
};

#endif
