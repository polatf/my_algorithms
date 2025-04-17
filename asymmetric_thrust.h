#ifndef ASYMMETRIC_THRUST_H
#define ASYMMETRIC_THRUST_H

// İki motor thrust bilgisini tutan yapı
struct MotorThrust {
    double left;   // Sol motor thrust (Newton)
    double right;  // Sağ motor thrust (Newton)
};

// Sağ ve sol motor thrust farkından doğan roll momenti hesaplayan sınıf
class AsymmetricThrustModel {
private:
    // d: motorlar arası mesafenin yarısı (örneğin 5m aralıklı motorlar için 2.5m)
    double motor_distance_half = 2.5;

public:
    // Moment = (T_right - T_left) × d
    double computeRollingMoment(const MotorThrust& thrust) {
        return (thrust.right - thrust.left) * motor_distance_half;
    }
};

#endif
