#include "algorithms/planar_imu_integrator.hpp"
#include <numeric>
#include <cmath>

namespace algorithms {

    void PlanarImuIntegrator::update(float gyro_z, double dt) {
        // Odstránenie odhadu biasu (offset)
        float corrected_gyro = gyro_z - gyro_offset_;
        theta_ += corrected_gyro * static_cast<float>(dt);
    }

    void PlanarImuIntegrator::setCalibration(std::vector<float> gyro) {
        if (gyro.empty()) {
            gyro_offset_ = 0.0f;
            return;
        }

        float sum = std::accumulate(gyro.begin(), gyro.end(), 0.0f);
        gyro_offset_ = sum / static_cast<float>(gyro.size());
    }

    float PlanarImuIntegrator::getYaw() const {
        return theta_;
    }

    void PlanarImuIntegrator::reset() {
        theta_ = 0.0f;
        gyro_offset_ = 0.0f;
    }

} // namespace algorithms
