#pragma once

#include "Arduino.h"
#include "esc.hpp"
#include "Servo.h"

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class TinyKart {
    Servo servo{};
    ESC esc;
    bool estopped;

    /// Kart wheelbase in meters
    float wheelbase;
    /// Max steering angle in degrees, symmetrical
    float max_steering;
    /// Percent to cap speed to
    float speed_cap;
    /// Biases the steering (trim)
    float steering_bias;

public:
    /// Prepare the kart for motion. toggle_pause must be called before we can move
    explicit TinyKart(int servo_pin,
                      const ESC &esc,
                      float speed_cap,
                      float steering_bias = 0.0,
                      float wheelbase = 0.335,
                      float max_steering = 24.0) : esc(esc),
                                                   wheelbase(wheelbase),
                                                   max_steering(max_steering),
                                                   speed_cap(speed_cap),
                                                   steering_bias(steering_bias) {
        // Prime actuators
        servo.attach(servo_pin, 0, 4096, map(50, 0, 4092, 0, 100));
        this->set_steering(0);
        this->esc.arm();

        // Start the kart stopped
        this->estopped = true;
    }

    /// Sets the motor to neutral.
    void set_neutral() {
        this->esc.set_neutral();
    }

    /// Sets the esc to power forward with some 0.0-1.0 percent.
    void set_forward(float power) {
        if (this->estopped) {
            this->set_neutral();
            return;
        }

        power = std::clamp(power, 0.0f, speed_cap);

        this->esc.set_forward(power);
    }

    /// Sets the esc to power reverse with some 0.0-1.0 percent.
    void set_reverse(float power) {
        if (this->estopped) {
            this->set_neutral();
            return;
        }

        power = std::clamp(power, 0.0f, speed_cap);

        this->esc.set_reverse(power);
    }

    /// Moves the servo such that the 'virtual' Ackermann wheel is at the passed angle.
    ///
    /// Positive angles are to the left, negative is to the right. All angles are in degrees.
    void set_steering(float angle) {
        if (this->estopped) return;

        angle = std::clamp(angle, -get_max_steering(), get_max_steering());

        // Max steering is 24 degrees, map to servo 0-180 degrees
        auto servo_angle = mapfloat(angle + steering_bias, -get_max_steering(), get_max_steering(), 199, 0);
        servo.write(int(servo_angle));
    }

    /// If set to high, stops the kart and prevents moving until called again. Starts high.
    void toggle_pause() {
        this->set_steering(0);
        this->set_neutral();

        this->estopped = !this->estopped;
    }

    /// Stops all kart actuations.
    void pause() {
        this->set_steering(0);
        this->set_neutral();

        this->estopped = true;
    }

    void unpause() {
        this->estopped = false;
    }

    /// Kart wheelbase in meters.
    [[nodiscard]] float get_wheelbase() const {
        return this->wheelbase;
    }

    /// Max steering angle in degrees, symmetrical
    [[nodiscard]] float get_max_steering() const {
        return this->max_steering;
    }

    /// Percent to cap speed to
    [[nodiscard]] float get_speed_cap() const {
        return this->speed_cap;
    }
};