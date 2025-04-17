#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// PID kontrolörü 
class PID {
private:
    double Kp, Ki, Kd;     // Kazançlar (proportional, integral, derivative)
    double integral;       // Hata toplamı (için)
    double prev_error;     // Bir önceki hata (d için)

public:
    // Yapıcı: kazançlar atanır, başlatmalar yapılır
    PID(double kp, double ki, double kd)
        : Kp(kp), Ki(ki), Kd(kd), integral(0.0), prev_error(0.0) {}

    // Her adımda kontrol sinyali üretir
    double calculate(double setpoint, double measured, double dt) {
        double error = setpoint - measured;              // Anlık hata hesapla
        integral += error * dt;                         // Toplam hatayı biriktir
        double derivative = (error - prev_error) / dt; // Hata değişim hızı

        prev_error = error;                           // Önceki hatayı güncelle

        // PID formülü uygulanır
        double output = Kp * error + Ki * integral + Kd * derivative;
        return output;
    }

    // PID reset fonksiyonu 
    void reset() {
        integral = 0.0;
        prev_error = 0.0;
    }
};

#endif
