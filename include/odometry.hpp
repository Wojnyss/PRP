#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <cmath>

// Struktura představující aktuální polohu robota.
struct Pose {
    double x;      // souřadnice x (v metrech)
    double y;      // souřadnice y (v metrech)
    double theta;  // orientace (v radiánech)
};

// Třída pro výpočet odometrie.
class Odometry {
public:
    // Konstruktor nastaví průměr kola, rozteč kol a počet pulzů na jeden oběh.
    Odometry(double wheel_diameter, double wheel_base, int pulses_per_rev);

    // Funkce pro aktualizaci polohy na základě počtu pulzů z obou enkodérů.
    void update(int pulses_left, int pulses_right);

    // Vrací aktuální polohu.
    Pose getPose() const;

    // Resetuje polohu na počátek.
    void resetPose();

private:
    double wheel_diameter_; // průměr kola (v metrech)
    double wheel_base_;     // rozteč kol (v metrech)
    int pulses_per_rev_;    // počet pulzů za jeden oběh kola
    Pose current_pose_;     // aktuální polohové souřadnice

    // (Volitelně) interní pomocná funkce pro normalizaci úhlu.
    double normalizeAngle(double angle);
};

#endif // ODOMETRY_HPP
