#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <cmath>
#include <csignal>
#include <algorithm> // For std::clamp
#include <memory>    // For std::unique_ptr
#include <stdexcept> // For exceptions


// Motor Pin Definitions (BCM GPIO numbering)
#define LEFT_MOTOR_PWM 17  // GPIO17 - Left motors PWM (speed)
#define LEFT_MOTOR_DIR 27  // GPIO27 - Left motors direction (HIGH=forward, LOW=backward)
#define RIGHT_MOTOR_PWM 22 // GPIO22 - Right motors PWM (speed)
#define RIGHT_MOTOR_DIR 23 // GPIO23 - Right motors direction (HIGH=forward, LOW=backward)

// Controller Configuration
#define JOYSTICK_DEVICE "/dev/input/js0"
#define DEADZONE 0.15f          // 15% deadzone for stick and triggers
#define MAX_PWM 100             // Maximum PWM value (0-100 for softPwm)
#define TURN_AGGRESSION 0.7f    // How aggressive the turning is (0.0-1.0)
#define CONTROL_LOOP_DELAY 10000 // 10ms = 100Hz update rate

// Xbox One Controller Button/Axis Mapping (standard Linux driver)
#define AXIS_LEFT_STICK_X 0
#define AXIS_LEFT_TRIGGER 2
#define AXIS_RIGHT_TRIGGER 5

// Volatile signal flag for clean loop termination
volatile sig_atomic_t running = 1;


/**
 * @brief Applies a deadzone to an analog input value and scales the result.
 * @param value The raw input value (-1.0 to 1.0 or 0.0 to 1.0).
 * @param deadzone The deadzone threshold (0.0 to 1.0).
 * @return float The scaled value, or 0.0 if within the deadzone.
 */
float applyDeadzone(float value, float deadzone) {
    if (std::abs(value) < deadzone) {
        return 0.0f;
    }
    // Scale the remaining range to 0-1 (retaining sign)
    float sign = (value > 0) ? 1.0f : -1.0f;
    return sign * ((std::abs(value) - deadzone) / (1.0f - deadzone));
}

class MotorController {
public:
    MotorController() {
        // Use BCM GPIO numbering
        if (wiringPiSetupGpio() == -1) {
            throw std::runtime_error("Failed to initialize wiringPi.");
        }

        // Setup PWM pins and initialize to 0 speed
        softPwmCreate(LEFT_MOTOR_PWM, 0, MAX_PWM);
        softPwmCreate(RIGHT_MOTOR_PWM, 0, MAX_PWM);

        // Setup direction pins as outputs
        pinMode(LEFT_MOTOR_DIR, OUTPUT);
        pinMode(RIGHT_MOTOR_DIR, OUTPUT);

        // Initialize to a defined state (e.g., forward direction, stopped)
        digitalWrite(LEFT_MOTOR_DIR, HIGH);
        digitalWrite(RIGHT_MOTOR_DIR, HIGH);
        stopAllMotors();
        
        std::cout << "Motors initialized on GPIO pins." << std::endl;
    }

    /**
     * @brief Stops all motors by setting PWM to 0.
     */
    void stopAllMotors() {
        softPwmWrite(LEFT_MOTOR_PWM, 0);
        softPwmWrite(RIGHT_MOTOR_PWM, 0);
    }

    /**
     * @brief Sets motor speeds using proportional differential steering (Tank drive).
     * @param throttle Net forward/backward speed (-1.0 to 1.0).
     * @param steering Net left/right steering (-1.0 to 1.0).
     */
    void setMotorSpeed(float throttle, float steering) {
        // Clamp inputs to ensure they are within the expected -1.0 to 1.0 range
        throttle = std::clamp(throttle, -1.0f, 1.0f);
        steering = std::clamp(steering, -1.0f, 1.0f);

        float leftSpeed, rightSpeed;
        
        // Differential steering calculation:
        // When turning, reduce power to the inside wheel proportionally to the steering amount.
        
        if (steering < 0) { // Turning Left (reduce left speed, right speed is full throttle)
            leftSpeed = throttle * (1.0f + steering * TURN_AGGRESSION);
            rightSpeed = throttle;
        } else if (steering > 0) { // Turning Right (reduce right speed, left speed is full throttle)
            leftSpeed = throttle;
            rightSpeed = throttle * (1.0f - steering * TURN_AGGRESSION);
        } else { // Moving Straight
            leftSpeed = throttle;
            rightSpeed = throttle;
        }

        // Final clamp to ensure no overflow
        leftSpeed = std::clamp(leftSpeed, -1.0f, 1.0f);
        rightSpeed = std::clamp(rightSpeed, -1.0f, 1.0f);

        // Convert proportional speed (-1.0 to 1.0) to absolute PWM value (0-100)
        int leftPWM = static_cast<int>(std::abs(leftSpeed) * MAX_PWM);
        int rightPWM = static_cast<int>(std::abs(rightSpeed) * MAX_PWM);

        // Set direction (HIGH for forward/positive speed, LOW for backward/negative speed)
        digitalWrite(LEFT_MOTOR_DIR, leftSpeed >= 0 ? HIGH : LOW);
        softPwmWrite(LEFT_MOTOR_PWM, leftPWM);

        digitalWrite(RIGHT_MOTOR_DIR, rightSpeed >= 0 ? HIGH : LOW);
        softPwmWrite(RIGHT_MOTOR_PWM, rightPWM);
    }
};


void signalHandler(int signum) {
    if (signum == SIGINT) {
        std::cout << "\nShutting down rover safely..." << std::endl;
        running = 0;
    }
}


int main() {
    std::cout << "=== Mars Rover Control System ===" << std::endl;
    
    // Setup signal handler for Ctrl+C (SIGINT)
    signal(SIGINT, signalHandler);

    try {
        // Initialize motor hardware
        MotorController motors;

        // Custom deleter for file descriptor to ensure closure (RAII)
        auto file_closer = [](int* fd) {
            if (*fd >= 0) {
                close(*fd);
            }
            delete fd; // Delete the pointer created with 'new'
        };
        // Open joystick device (using smart pointer for automatic cleanup)
        std::unique_ptr<int, decltype(file_closer)> joystick_fd_ptr(
            new int(open(JOYSTICK_DEVICE, O_RDONLY | O_NONBLOCK)), file_closer
        );
        
        int joystick_fd = *joystick_fd_ptr;

        if (joystick_fd < 0) {
            std::cerr << "Error: Could not open joystick device " << JOYSTICK_DEVICE << std::endl;
            std::cerr << "Make sure your Xbox controller is connected and the 'joystick' kernel module is loaded." << std::endl;
            return 1;
        }
        std::cout << "Xbox controller connected successfully." << std::endl;

        std::cout << "\nRover ready! " << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "* **Right Trigger:** Forward (Proportional Speed)" << std::endl;
        std::cout << "* **Left Trigger:** Backward (Proportional Speed)" << std::endl;
        std::cout << "* **Left Stick (X-axis):** Steering" << std::endl;
        std::cout << "Press Ctrl+C to exit.\n" << std::endl;

        // Controller state storage
        float rightTrigger = 0.0f;  // 0.0 to 1.0 (Forward)
        float leftTrigger = 0.0f;   // 0.0 to 1.0 (Backward)
        float steeringX = 0.0f;     // -1.0 to 1.0 (Left/Right)

        struct js_event event;

        // Main control loop
        while (running) {
            // Read all pending controller events in the buffer
            while (read(joystick_fd, &event, sizeof(event)) > 0) {
                // Process only axis events (sticks and triggers)
                if (event.type == JS_EVENT_AXIS) {
                    // Normalize event value from native range (-32767 to 32767) to -1.0 to 1.0
                    float value = event.value / 32767.0f;

                    switch (event.number) {
                        case AXIS_LEFT_STICK_X:
                            steeringX = applyDeadzone(value, DEADZONE);
                            break;

                        case AXIS_RIGHT_TRIGGER:
                            // Triggers are usually -1.0 (off) to 1.0 (full). 
                            // Convert to 0.0 (off) to 1.0 (full) for proportional speed.
                            rightTrigger = applyDeadzone((value + 1.0f) / 2.0f, DEADZONE);
                            break;

                        case AXIS_LEFT_TRIGGER:
                            leftTrigger = applyDeadzone((value + 1.0f) / 2.0f, DEADZONE);
                            break;
                    }
                }
            }

            // Calculate net throttle (speed and direction): 
            // Positive for forward, Negative for reverse.
            float throttle = rightTrigger - leftTrigger;

            // Apply motor control with differential steering
            motors.setMotorSpeed(throttle, steeringX);

            // Small delay to ensure the process doesn't hog the CPU
            usleep(CONTROL_LOOP_DELAY);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "A critical error occurred: " << e.what() << std::endl;
        // Ensure motors are stopped on error
        MotorController motors;
        motors.stopAllMotors();
        return 1;
    }

    std::cout << "Rover shutdown complete." << std::endl;
    return 0;
}
