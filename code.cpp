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

/**
 * @brief Controls two motors using an L298N H-bridge module (6-pin mode).
 * Uses three pins per motor: ENA (PWM), IN1, and IN2 (direction).
 */
class MotorController {
private:
    int leftENA;   // PWM pin for left motor speed control
    int leftIN1;   // Left direction pin 1
    int leftIN2;   // Left direction pin 2
    int rightENB;  // PWM pin for right motor speed control
    int rightIN3;  // Right direction pin 3
    int rightIN4;  // Right direction pin 4
    bool initialized;

public:
    // Constructor initializes pins
    MotorController(int lENA, int lIN1, int lIN2, int rENB, int rIN3, int rIN4)
        : leftENA(lENA), leftIN1(lIN1), leftIN2(lIN2),
          rightENB(rENB), rightIN3(rIN3), rightIN4(rIN4),
          initialized(false) {}

    // Destructor ensures GPIO cleanup
    ~MotorController() {
        if (initialized) {
            stop();
            gpioTerminate();
        }
    }

    // Initialize pigpio and set up all 6 pins
    bool initialize() {
        if (gpioInitialise() < 0) {
            std::cerr << "Failed to initialize pigpio library" << std::endl;
            return false;
        }

        // Configure GPIO pins as outputs
        gpioSetMode(leftENA, PI_OUTPUT);
        gpioSetMode(leftIN1, PI_OUTPUT);
        gpioSetMode(leftIN2, PI_OUTPUT);

        gpioSetMode(rightENB, PI_OUTPUT);
        gpioSetMode(rightIN3, PI_OUTPUT);
        gpioSetMode(rightIN4, PI_OUTPUT);

        // PWM setup (frequency and range)
        gpioSetPWMfrequency(leftENA, PWM_FREQUENCY);
        gpioSetPWMfrequency(rightENB, PWM_FREQUENCY);
        gpioSetPWMrange(leftENA, PWM_RANGE);
        gpioSetPWMrange(rightENB, PWM_RANGE);

        // Initialize to stopped state (PWM=0, all direction pins LOW)
        gpioPWM(leftENA, 0);
        gpioPWM(rightENB, 0);
        gpioWrite(leftIN1, 0);
        gpioWrite(leftIN2, 0);
        gpioWrite(rightIN3, 0);
        gpioWrite(rightIN4, 0);

        initialized = true;
        std::cout << " Motors initialized using L298N (6-pin mode)" << std::endl;
        std::cout << "   Left:  ENA=" << leftENA << " IN1=" << leftIN1 << " IN2=" << leftIN2 << std::endl;
        std::cout << "   Right: ENB=" << rightENB << " IN3=" << rightIN3 << " IN4=" << rightIN4 << std::endl;
        return true;
    }

    /**
     * @brief Calculates differential steering and applies speed and direction to motors.
     * @param throttle Overall speed [-1.0 (backward) to 1.0 (forward)]
     * @param steering Steering input [-1.0 (left) to 1.0 (right)]
     */
    void setSpeed(float throttle, float steering) {
        if (!initialized) return;

        throttle = std::clamp(throttle, -1.0f, 1.0f);
        steering = std::clamp(steering, -1.0f, 1.0f);

        float leftSpeed, rightSpeed;

        // Differential steering logic
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

        // Convert speed (magnitude) to PWM values (0 to PWM_RANGE)
        int leftPWM = static_cast<int>(std::abs(leftSpeed) * PWM_RANGE);
        int rightPWM = static_cast<int>(std::abs(rightSpeed) * PWM_RANGE);

        // --- Left Motor Control ---
        // Forward: IN1=1, IN2=0
        // Backward: IN1=0, IN2=1
        gpioWrite(leftIN1, leftSpeed >= 0 ? 1 : 0);
        gpioWrite(leftIN2, leftSpeed >= 0 ? 0 : 1);
        gpioPWM(leftENA, leftPWM);

        // --- Right Motor Control ---
        // Forward: IN3=1, IN4=0
        // Backward: IN3=0, IN4=1
        gpioWrite(rightIN3, rightSpeed >= 0 ? 1 : 0);
        gpioWrite(rightIN4, rightSpeed >= 0 ? 0 : 1);
        gpioPWM(rightENB, rightPWM);
    }

    // Stop both motors (set PWM to 0 and direction pins LOW)
    void stop() {
        if (initialized) {
            gpioPWM(leftENA, 0);
            gpioPWM(rightENB, 0);
            gpioWrite(leftIN1, 0);
            gpioWrite(leftIN2, 0);
            gpioWrite(rightIN3, 0);
            gpioWrite(rightIN4, 0);
        }
    }
};


/**
 * @brief Handles reading and processing input from a connected Xbox controller.
 */
class XboxController {
private:
    int fileDescriptor;
    float rightTrigger;
    float leftTrigger;
    float steeringX;
    float deadzone;

    // Helper function to apply a deadzone and scale the input
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

    // Initialize by opening the joystick device file
    bool initialize(const char* device = JOYSTICK_DEVICE) {
        fileDescriptor = open(device, O_RDONLY | O_NONBLOCK);
        if (fileDescriptor < 0) {
            std::cerr << "  Error: Could not open joystick device " << device << std::endl;
            std::cerr << "   Make sure your Xbox controller is connected" << std::endl;
            std::cerr << "   Check with: ls -l /dev/input/js*" << std::endl;
            return false;
        }
        std::cout << " Xbox controller connected successfully" << std::endl;
        return true;
    }

    // Read and process controller events from the device file
    void update() {
        if (fileDescriptor < 0) return;

        struct js_event event;
        while (read(fileDescriptor, &event, sizeof(event)) > 0) {
            if (event.type == JS_EVENT_AXIS) {
                // Joystick input is typically -32767 to 32767, normalize to -1.0 to 1.0
                float value = event.value / 32767.0f;

                switch (event.number) {
                    case AXIS_LEFT_STICK_X:
                        steeringX = applyDeadzone(value);
                        break;

                    case AXIS_RIGHT_TRIGGER:
                        // Convert trigger from -1..1 to 0..1 range
                        rightTrigger = applyDeadzone((value + 1.0f) / 2.0f);
                        break;

                    case AXIS_LEFT_TRIGGER:
                        leftTrigger = applyDeadzone((value + 1.0f) / 2.0f);
                        break;
                }
            }
        }
    }

    // Calculates net throttle (Forward - Backward)
    float getThrottle() const {
        return rightTrigger - leftTrigger;  // -1.0 (full reverse) to 1.0 (full forward)
    }

    // Returns steering input
    float getSteering() const {
        return steeringX;  // -1.0 (full left) to 1.0 (full right)
    }
};


/**
 * @brief Main class orchestrating the rover's control and hardware components.
 */
class RoverSystem {
private:
    std::unique_ptr<MotorController> motors;
    std::unique_ptr<XboxController> controller;

public:
    // RoverSystem constructor now takes 6 pins for L298N
    RoverSystem(int lENA, int lIN1, int lIN2,
                int rENB, int rIN3, int rIN4) {
        motors = std::make_unique<MotorController>(lENA, lIN1, lIN2, rENB, rIN3, rIN4);
        controller = std::make_unique<XboxController>();
    }

    // Initializes motors and controller
    bool initialize() {
        std::cout << "=== Mars Rover Control System ===" << std::endl;
        std::cout << "Initializing hardware..." << std::endl;

        if (!motors->initialize()) {
            std::cerr << " Failed to initialize motors" << std::endl;
            return false;
        }

        if (!controller->initialize()) {
            std::cerr << "Failed to initialize controller" << std::endl;
            return false;
        }

        std::cout << "\n Rover ready! (L298N 6-Pin Mode)" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  Right Trigger: Forward" << std::endl;
        std::cout << "  Left Trigger:  Backward" << std::endl;
        std::cout << "  Left Stick X:  Steering" << std::endl;
        std::cout << "  Press Ctrl+C to exit\n" << std::endl;

        return true;
    }

    // The main control loop
    void run() {
        while (running) {
            // Read controller input
            controller->update();

            // Get throttle and steering values
            float throttle = controller->getThrottle();
            float steering = controller->getSteering();

            // Control motors with differential steering
            motors->setSpeed(throttle, steering);

            // Small delay to prevent CPU overload and keep loop rate consistent
            usleep(CONTROL_LOOP_DELAY);
        }
        shutdown();
    }

private:
    // Safely stop motors before termination
    void shutdown() {
        std::cout << "Stopping motors..." << std::endl;
        motors->stop();
        std::cout << " Rover shutdown complete" << std::endl;
    }
};


int main() {
    // Setup signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    try {
        // GPIO Pin Configuration (BCM numbering) for L298N H-Bridge (6 pins)
        // These pins match the definitions provided in your request snippet.
        const int LEFT_ENA = 22;
        const int LEFT_IN1 = 17;
        const int LEFT_IN2 = 27;

        const int RIGHT_ENB = 25;
        const int RIGHT_IN3 = 23;
        const int RIGHT_IN4 = 24;

        // Create and initialize rover system
        RoverSystem rover(LEFT_ENA, LEFT_IN1, LEFT_IN2,
                         RIGHT_ENB, RIGHT_IN3, RIGHT_IN4);

        if (!rover.initialize()) {
            return 1;
        }

        // Run main control loop
        rover.run();

    } catch (const std::exception& e) {
        std::cerr << " Critical error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

