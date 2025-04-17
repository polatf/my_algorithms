#ifndef ROC_EVALUATOR_H
#define ROC_EVALUATOR_H

// ROC: Rate of Climb (m/s) - Tırmanma hızı calculator
class ROCEvaluator {
private:
    double previousAltitude;  // Bir önce ölçülen irtifa

public:
    // Başlangıç değeri 
    ROCEvaluator() : previousAltitude(0.0) {}

    // ROC = (yükseklik değişimi) / (zaman adımı)
    double computeROC(double currentAltitude, double dt) {
        double roc = (currentAltitude - previousAltitude) / dt;
        previousAltitude = currentAltitude;
        return roc;
    }

    // FSM geçişlerinde sıfırlama 
    void reset() {
        previousAltitude = 0.0;
    }
};

#endif
