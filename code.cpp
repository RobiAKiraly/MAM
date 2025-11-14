#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <pigpio.h>
#include <cmath>
#include <csignal>
#include <algorithm>
#include <memory>
#include <stdexcept>

// Configuration Constants
#define JOYSTICK_DEVICE "/dev/input/js0"
#define DEADZONE 0.15f
#define PWM_FREQUENCY 1000
#define PWM_RANGE 255
#define TURN_AGGRESSION 0.7f
#define CONTROL_LOOP_DELAY 10000  // 10ms = 100Hz

// Xbox Controller Axis Mapping
#define AXIS_LEFT_STICK_X 0
#define AXIS_LEFT_TRIGGER 2
#define AXIS_RIGHT_TRIGGER 5

// Global flag for signal handling
volatile sig_atomic_t running = 1;

// Signal handler
void signalHandler(int signum) {
    std::cout << "\nShutting down rover safely..." << std::endl;
    running = 0;
}


class MotorController {
private:
    int leftPwmPin;
    int leftDirPin;
    int rightPwmPin;
    int rightDirPin;
    bool initialized;

public:
    MotorController(int leftPWM, int leftDIR, int rightPWM, int rightDIR)
        : leftPwmPin(leftPWM), leftDirPin(leftDIR),
          rightPwmPin(rightPWM), rightDirPin(rightDIR),
          initialized(false) {}

    ~MotorController() {
        if (initialized) {
            stop();
            gpioTerminate();
        }
    }

    bool initialize() {
        if (gpioInitialise() < 0) {
            std::cerr << "Failed to initialize pigpio library" << std::endl;
            return false;
        }

        // Configure GPIO pins
        gpioSetMode(leftPwmPin, PI_OUTPUT);
        gpioSetMode(leftDirPin, PI_OUTPUT);
        gpioSetMode(rightPwmPin, PI_OUTPUT);
        gpioSetMode(rightDirPin, PI_OUTPUT);

        // Set PWM frequency and range
        gpioSetPWMfrequency(leftPwmPin, PWM_FREQUENCY);
        gpioSetPWMfrequency(rightPwmPin, PWM_FREQUENCY);
        gpioSetPWMrange(leftPwmPin, PWM_RANGE);
        gpioSetPWMrange(rightPwmPin, PWM_RANGE);

        // Initialize to stopped state
        gpioWrite(leftDirPin, 1);
        gpioWrite(rightDirPin, 1);
        gpioPWM(leftPwmPin, 0);
        gpioPWM(rightPwmPin, 0);

        initialized = true;
        std::cout << "✅ Motors initialized on GPIO pins" << std::endl;
        std::cout << "   Left:  PWM=" << leftPwmPin << " DIR=" << leftDirPin << std::endl;
        std::cout << "   Right: PWM=" << rightPwmPin << " DIR=" << rightDirPin << std::endl;
        return true;
    }

    void stop() {
        if (initialized) {
            gpioPWM(leftPwmPin, 0);
            gpioPWM(rightPwmPin, 0);
        }
    }

    void setSpeed(float throttle, float steering) {
        if (!initialized) return;

        // Clamp inputs to valid range
        throttle = std::clamp(throttle, -1.0f, 1.0f);
        steering = std::clamp(steering, -1.0f, 1.0f);

        // Calculate differential steering
        float leftSpeed, rightSpeed;

        if (steering < 0) {
            // Turning left - reduce left motor speed
            leftSpeed = throttle * (1.0f + steering * TURN_AGGRESSION);
            rightSpeed = throttle;
        } else if (steering > 0) {
            // Turning right - reduce right motor speed
            leftSpeed = throttle;
            rightSpeed = throttle * (1.0f - steering * TURN_AGGRESSION);
        } else {
            // Straight movement
            leftSpeed = throttle;
            rightSpeed = throttle;
        }

        // Final clamp to ensure valid range
        leftSpeed = std::clamp(leftSpeed, -1.0f, 1.0f);
        rightSpeed = std::clamp(rightSpeed, -1.0f, 1.0f);

        // Convert to PWM values
        int leftPWM = static_cast<int>(std::abs(leftSpeed) * PWM_RANGE);
        int rightPWM = static_cast<int>(std::abs(rightSpeed) * PWM_RANGE);

        // Set direction and speed
        gpioWrite(leftDirPin, leftSpeed >= 0 ? 1 : 0);
        gpioPWM(leftPwmPin, leftPWM);

        gpioWrite(rightDirPin, rightSpeed >= 0 ? 1 : 0);
        gpioPWM(rightPwmPin, rightPWM);
    }
};


class XboxController {
private:
    int fileDescriptor;
    float rightTrigger;
    float leftTrigger;
    float steeringX;
    float deadzone;

    float applyDeadzone(float value) {
        if (std::abs(value) < deadzone) {
            return 0.0f;
        }
        float sign = (value > 0) ? 1.0f : -1.0f;
        return sign * ((std::abs(value) - deadzone) / (1.0f - deadzone));
    }

public:
    XboxController(float deadzoneThreshold = DEADZONE)
        : fileDescriptor(-1), rightTrigger(0.0f), leftTrigger(0.0f),
          steeringX(0.0f), deadzone(deadzoneThreshold) {}

    ~XboxController() {
        if (fileDescriptor >= 0) {
            close(fileDescriptor);
        }
    }

    bool initialize(const char* device = JOYSTICK_DEVICE) {
        fileDescriptor = open(device, O_RDONLY | O_NONBLOCK);
        if (fileDescriptor < 0) {
            std::cerr << "❌ Error: Could not open joystick device " << device << std::endl;
            std::cerr << "   Make sure your Xbox controller is connected" << std::endl;
            std::cerr << "   Check with: ls -l /dev/input/js*" << std::endl;
            return false;
        }
        std::cout << "✅ Xbox controller connected successfully" << std::endl;
        return true;
    }

    void update() {
        if (fileDescriptor < 0) return;

        struct js_event event;
        while (read(fileDescriptor, &event, sizeof(event)) > 0) {
            if (event.type == JS_EVENT_AXIS) {
                float value = event.value / 32767.0f;

                switch (event.number) {
                    case AXIS_LEFT_STICK_X:
                        steeringX = applyDeadzone(value);
                        break;

                    case AXIS_RIGHT_TRIGGER:
                        // Convert from -1..1 to 0..1 range
                        rightTrigger = applyDeadzone((value + 1.0f) / 2.0f);
                        break;

                    case AXIS_LEFT_TRIGGER:
                        leftTrigger = applyDeadzone((value + 1.0f) / 2.0f);
                        break;
                }
            }
        }
    }

    float getThrottle() const {
        return rightTrigger - leftTrigger;  // -1.0 to 1.0
    }

    float getSteering() const {
        return steeringX;  // -1.0 to 1.0
    }

    float getRightTrigger() const { return rightTrigger; }
    float getLeftTrigger() const { return leftTrigger; }
};


class RoverSystem {
private:
    std::unique_ptr<MotorController> motors;
    std::unique_ptr<XboxController> controller;

public:
    RoverSystem(int leftPWM, int leftDIR, int rightPWM, int rightDIR) {
        motors = std::make_unique<MotorController>(leftPWM, leftDIR, rightPWM, rightDIR);
        controller = std::make_unique<XboxController>();
    }

    bool initialize() {
        std::cout << "=== Mars Rover Control System ===" << std::endl;
        std::cout << "Initializing hardware..." << std::endl;

        if (!motors->initialize()) {
            std::cerr << "❌ Failed to initialize motors" << std::endl;
            return false;
        }

        if (!controller->initialize()) {
            std::cerr << "❌ Failed to initialize controller" << std::endl;
            return false;
        }

        std::cout << "\n🚀 Rover ready!" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  Right Trigger: Forward" << std::endl;
        std::cout << "  Left Trigger:  Backward" << std::endl;
        std::cout << "  Left Stick X:  Steering" << std::endl;
        std::cout << "  Press Ctrl+C to exit\n" << std::endl;

        return true;
    }

    void run() {
        while (running) {
            // Read controller input
            controller->update();

            // Get throttle and steering values
            float throttle = controller->getThrottle();
            float steering = controller->getSteering();

            // Control motors with differential steering
            motors->setSpeed(throttle, steering);

            // Small delay to prevent CPU overload
            usleep(CONTROL_LOOP_DELAY);
        }
    }

    void shutdown() {
        std::cout << "Stopping motors..." << std::endl;
        motors->stop();
        std::cout << "✅ Rover shutdown complete" << std::endl;
    }
};


int main() {
    // Setup signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    try {
        // GPIO Pin Configuration (BCM numbering)
        const int LEFT_MOTOR_PWM = 27;
        const int LEFT_MOTOR_DIR = 5;
        const int RIGHT_MOTOR_PWM = 6;
        const int RIGHT_MOTOR_DIR = 13;

        // Create and initialize rover system
        RoverSystem rover(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR, 
                         RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR);

        if (!rover.initialize()) {
            return 1;
        }

        // Run main control loop
        rover.run();

        // Clean shutdown
        rover.shutdown();

    } catch (const std::exception& e) {
        std::cerr << "❌ Critical error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

