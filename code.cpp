#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <pigpio.h>
#include <cmath>
#include <csignal>
#include <algorithm>

// Motor Pin Definitions (BCM GPIO numbering)
#define LEFT_MOTOR_PWM 27        // GPIO27 - Left motors PWM (speed)
#define LEFT_MOTOR_DIR 5        // GPIO5 - Left motors direction (HIGH=forward, LOW=backward)
#define RIGHT_MOTOR_PWM 6       // GPIO6 - Right motors PWM (speed)
#define RIGHT_MOTOR_DIR 13       // GPIO13 - Right motors direction (HIGH=forward, LOW=backward)

// Controller Configuration
#define JOYSTICK_DEVICE "/dev/input/js0"
#define DEADZONE 0.15f           // 15% deadzone for stick and triggers
#define PWM_FREQUENCY 1000       // 1kHz PWM frequency
#define PWM_RANGE 255            // PWM range (0-255)
#define TURN_AGGRESSION 0.7f     // How aggressive the turning is (0.0-1.0)

// Xbox One Controller Axis Mapping
#define AXIS_LEFT_STICK_X 0
#define AXIS_LEFT_TRIGGER 2
#define AXIS_RIGHT_TRIGGER 5

// Global variables
volatile sig_atomic_t running = 1;
int joystick_fd = -1;

// Signal handler for clean shutdown
void signalHandler(int signum) {
    std::cout << "\nShutting down rover safely..." << std::endl;
    running = 0;
}

// Initialize GPIO pins for motors using pigpio
bool initMotors() {
    // Initialize pigpio library
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio library" << std::endl;
        return false;
    }
    
    // Set GPIO modes
    gpioSetMode(LEFT_MOTOR_PWM, PI_OUTPUT);
    gpioSetMode(LEFT_MOTOR_DIR, PI_OUTPUT);
    gpioSetMode(RIGHT_MOTOR_PWM, PI_OUTPUT);
    gpioSetMode(RIGHT_MOTOR_DIR, PI_OUTPUT);
    
    // Set PWM frequency (optional, default is usually fine)
    gpioSetPWMfrequency(LEFT_MOTOR_PWM, PWM_FREQUENCY);
    gpioSetPWMfrequency(RIGHT_MOTOR_PWM, PWM_FREQUENCY);
    
    // Set PWM range
    gpioSetPWMrange(LEFT_MOTOR_PWM, PWM_RANGE);
    gpioSetPWMrange(RIGHT_MOTOR_PWM, PWM_RANGE);
    
    // Initialize to forward direction, stopped
    gpioWrite(LEFT_MOTOR_DIR, 1);
    gpioWrite(RIGHT_MOTOR_DIR, 1);
    gpioPWM(LEFT_MOTOR_PWM, 0);
    gpioPWM(RIGHT_MOTOR_PWM, 0);
    
    std::cout << "Motors initialized successfully" << std::endl;
    return true;
}

// Stop all motors
void stopAllMotors() {
    gpioPWM(LEFT_MOTOR_PWM, 0);
    gpioPWM(RIGHT_MOTOR_PWM, 0);
}

// Apply deadzone to analog input
float applyDeadzone(float value, float deadzone) {
    if (std::abs(value) < deadzone) {
        return 0.0f;
    }
    float sign = (value > 0) ? 1.0f : -1.0f;
    return sign * ((std::abs(value) - deadzone) / (1.0f - deadzone));
}

// Set motor speeds with differential steering
void setMotorSpeed(float throttle, float steering) {
    // Clamp inputs
    throttle = std::clamp(throttle, -1.0f, 1.0f);
    steering = std::clamp(steering, -1.0f, 1.0f);
    
    float leftSpeed, rightSpeed;
    
    // Differential steering calculation
    if (steering < 0) {
        // Turning left
        leftSpeed = throttle * (1.0f + steering * TURN_AGGRESSION);
        rightSpeed = throttle;
    } else if (steering > 0) {
        // Turning right
        leftSpeed = throttle;
        rightSpeed = throttle * (1.0f - steering * TURN_AGGRESSION);
    } else {
        // Straight
        leftSpeed = throttle;
        rightSpeed = throttle;
    }
    
    // Final clamp
    leftSpeed = std::clamp(leftSpeed, -1.0f, 1.0f);
    rightSpeed = std::clamp(rightSpeed, -1.0f, 1.0f);
    
    // Convert to PWM values (0-255)
    int leftPWM = static_cast<int>(std::abs(leftSpeed) * PWM_RANGE);
    int rightPWM = static_cast<int>(std::abs(rightSpeed) * PWM_RANGE);
    
    // Set direction and speed for left motors
    gpioWrite(LEFT_MOTOR_DIR, leftSpeed >= 0 ? 1 : 0);
    gpioPWM(LEFT_MOTOR_PWM, leftPWM);
    
    // Set direction and speed for right motors
    gpioWrite(RIGHT_MOTOR_DIR, rightSpeed >= 0 ? 1 : 0);
    gpioPWM(RIGHT_MOTOR_PWM, rightPWM);
}

// Initialize Xbox controller
bool initController() {
    joystick_fd = open(JOYSTICK_DEVICE, O_RDONLY | O_NONBLOCK);
    if (joystick_fd < 0) {
        std::cerr << "Error: Could not open joystick device " << JOYSTICK_DEVICE << std::endl;
        std::cerr << "Make sure your Xbox controller is connected." << std::endl;
        std::cerr << "Check with: ls -l /dev/input/js*" << std::endl;
        return false;
    }
    std::cout << "Xbox controller connected successfully" << std::endl;
    return true;
}

int main() {
    std::cout << "=== Mars Rover Control System ===" << std::endl;
    std::cout << "Initializing..." << std::endl;
    
    // Setup signal handler for Ctrl+C
    signal(SIGINT, signalHandler);
    
    // Initialize hardware
    if (!initMotors()) {
        std::cerr << "Failed to initialize motors. Exiting." << std::endl;
        return 1;
    }
    
    if (!initController()) {
        std::cerr << "Failed to initialize controller. Exiting." << std::endl;
        gpioTerminate();
        return 1;
    }
    
    std::cout << "\n✅ Rover ready!" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Right Trigger: Forward" << std::endl;
    std::cout << "  Left Trigger: Backward" << std::endl;
    std::cout << "  Left Stick (X-axis): Steering" << std::endl;
    std::cout << "  Press Ctrl+C to exit\n" << std::endl;
    
    // Controller state
    float rightTrigger = 0.0f;
    float leftTrigger = 0.0f;
    float steeringX = 0.0f;
    
    struct js_event event;
    
    // Main control loop
    while (running) {
        // Read controller events
        while (read(joystick_fd, &event, sizeof(event)) > 0) {
            if (event.type == JS_EVENT_AXIS) {
                float value = event.value / 32767.0f;
                
                switch (event.number) {
                    case AXIS_LEFT_STICK_X:
                        steeringX = applyDeadzone(value, DEADZONE);
                        break;
                        
                    case AXIS_RIGHT_TRIGGER:
                        rightTrigger = applyDeadzone((value + 1.0f) / 2.0f, DEADZONE);
                        break;
                        
                    case AXIS_LEFT_TRIGGER:
                        leftTrigger = applyDeadzone((value + 1.0f) / 2.0f, DEADZONE);
                        break;
                }
            }
        }
        
        // Calculate throttle
        float throttle = rightTrigger - leftTrigger;
        
        // Apply motor control
        setMotorSpeed(throttle, steeringX);
        
        // Small delay (10ms = 100Hz)
        usleep(10000);
    }
    
    // Cleanup
    std::cout << "Stopping motors..." << std::endl;
    stopAllMotors();
    close(joystick_fd);
    gpioTerminate();
    std::cout << "Rover shutdown complete." << std::endl;
    
    return 0;
}
