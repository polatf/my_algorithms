#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// Kalman filtresi sınıfı - 1 boyutlu (örnek: pitch için)
class KalmanFilter {
private:
    double angle;       // Filtrelenmiş açı
    double bias;        // Gyro sapması (drift)
    double rate;        // Gyro ölçümünden düzeltilecek hız

    double P[2][2];     // Hata kovaryans matrisi
    double Q_angle;     // Açı süzgeç gürültü kovaryansı
    double Q_bias;      // Bias süzgeç gürültü kovaryansı
    double R_measure;   // Ölçüm gürültüsü kovaryansı

public:
    // başlangıç değerleri 
    KalmanFilter() {
        angle = bias = rate = 0.0;
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;

        P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0;
    }

    // Filtrelenmiş açı çıktısını üretir
    double getAngle(double acc_angle, double gyro_rate, double dt) {
        // ---- Prediction (Tahmin) Adımı ----
        rate = gyro_rate - bias;
        angle += dt * rate;

        // Hata kovaryans matrisi güncellenir
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // ---- Measurement Update (Düzeltme) Adımı ----
        double y = acc_angle - angle;          // Ölçüm ile tahmin farkı
        double S = P[0][0] + R_measure;        // Tahmini hata
        double K[2];                           // Kalman kazancı
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Tahmin düzeltmesi
        angle += K[0] * y;
        bias  += K[1] * y;

        // Hata kovaryans matrisi güncellenir
        double P00_temp = P[0][0];
        double P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
};

#endif
