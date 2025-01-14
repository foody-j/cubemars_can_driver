//motor_data.hpp
#ifndef MOTOR_DATA_HPP
#define MOTOR_DATA_HPP

#include <array>
#include <cstdint>

struct MotorData {
    float position{0.0f};    // -3200° ~ +3200°
    float speed{0.0f};       // -32000 ~ +32000 rpm
    float current{0.0f};     // -60A ~ +60A
    int8_t temperature{0};   // -20°C ~ 127°C
    uint8_t error{0};        // 0~7 error codes
};

class MotorDataManager {
public:
    static constexpr size_t MAX_MOTORS = 6;
    
    MotorData& getMotorData(uint8_t motor_id) { //
        if (motor_id < 1 || motor_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        return motor_data_[motor_id - 1];
    }

    void updateMotorData(uint8_t motor_id, const MotorData& data) {
        if (motor_id < 1 || motor_id > MAX_MOTORS) {
            throw std::runtime_error("Invalid motor ID");
        }
        motor_data_[motor_id - 1] = data;
    }

    void reset() {
        motor_data_.fill(MotorData{});
    }

private:
    std::array<MotorData, MAX_MOTORS> motor_data_;
};

#endif // MOTOR_DATA_HPP