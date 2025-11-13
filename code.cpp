#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <cmath>
#include <csignal>

// Motor Pin Definitions (adjust these to your actual GPIO pins)
// LEFT MOTORS
#define LEFT_MOTOR_PWM 17        // GPIO17 - Left motors PWM (speed)
#define LEFT_MOTOR_DIR 27        // GPIO27 - Left motors direction (HIGH=forward, LOW=backward)

// RIGHT MOTORS
#define RIGHT_MOTOR_PWM 22       // GPIO22 - Right motors PWM (speed)
#define RIGHT_MOTOR_DIR 23       // GPIO23 - Right motors direction (HIGH=forward, LOW=backward)

// Controller Configuration
#define JOYSTICK_DEVICE "/dev/input/js0"    
#define DEADZONE 0.15f           // 15% deadzone for stick and triggers
#define MAX_PWM 100              // Maximum PWM value (0-100)
#define TURN_AGGRESSION 0.7f     // How aggressive the turning is (0.0-1.0)

// Xbox One Controller Button/Axis Mapping (standard Linux driver)
#define AXIS_LEFT_STICK_X 0
#define AXIS_LEFT_STICK_Y 1
#define AXIS_LEFT_TRIGGER 2
#define AXIS_RIGHT_TRIGGER 5

// Global variables
bool running = true;
int joystick_fd = -1;

// Signal handler for clean shutdown
void signalHandler(int signum) {
    std::cout << "\nShutting down rover safely..." << std::endl;
    running = false;
}

// Initialize GPIO pins for motors
void initMotors() {
    wiringPiSetupGpio(); // Use BCM GPIO numbering
    
    // Setup PWM pins
    softPwmCreate(LEFT_MOTOR_PWM, 0, MAX_PWM);
    softPwmCreate(RIGHT_MOTOR_PWM, 0, MAX_PWM);
    
    // Setup direction pins as outputs
    pinMode(LEFT_MOTOR_DIR, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR, OUTPUT);
    
    // Initialize to forward direction
    digitalWrite(LEFT_MOTOR_DIR, HIGH);
    digitalWrite(RIGHT_MOTOR_DIR, HIGH);
    
    std::cout << "Motors initialized on GPIO pins" << std::endl;
}

// Stop all motors
void stopAllMotors() {
    softPwmWrite(LEFT_MOTOR_PWM, 0);
    softPwmWrite(RIGHT_MOTOR_PWM, 0);
}

// Apply deadzone to analog input
float applyDeadzone(float value, float deadzone) {
    if (std::abs(value) < deadzone) {
        return 0.0f;
    }
    // Scale the remaining range to 0-1
    float sign = (value > 0) ? 1.0f : -1.0f;
    return sign * ((std::abs(value) - deadzone) / (1.0f - deadzone));
}

// Set motor speeds with differential steering
void setMotorSpeed(float throttle, float steering) {
    // throttle: -1.0 (full backward) to 1.0 (full forward)
    // steering: -1.0 (full left) to 1.0 (full right)
    
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;
    
    // Calculate differential steering
    // When turning, reduce power to the inside wheels
    if (throttle >= 0) {
        // Forward movement
        leftSpeed = throttle;
        rightSpeed = throttle;
        
        if (steering < 0) {
            // Turning left - reduce left motor speed
            leftSpeed *= (1.0f + steering * TURN_AGGRESSION);
        } else if (steering > 0) {
            // Turning right - reduce right motor speed
            rightSpeed *= (1.0f - steering * TURN_AGGRESSION);
        }
    } else {
        // Backward movement
        leftSpeed = throttle;
        rightSpeed = throttle;
        
        if (steering < 0) {
            // Turning left while reversing - reduce left motor speed
            leftSpeed *= (1.0f + steering * TURN_AGGRESSION);
        } else if (steering > 0) {
            // Turning right while reversing - reduce right motor speed
            rightSpeed *= (1.0f - steering * TURN_AGGRESSION);
        }
    }
    
    // Clamp speeds to -1.0 to 1.0 range
    leftSpeed = std::max(-1.0f, std::min(1.0f, leftSpeed));
    rightSpeed = std::max(-1.0f, std::min(1.0f, rightSpeed));
    
    // Convert to PWM values and apply to motors
    int leftPWM = static_cast<int>(std::abs(leftSpeed) * MAX_PWM);
    int rightPWM = static_cast<int>(std::abs(rightSpeed) * MAX_PWM);
    
    // Left motors - set direction and speed
    digitalWrite(LEFT_MOTOR_DIR, leftSpeed >= 0 ? HIGH : LOW);
    softPwmWrite(LEFT_MOTOR_PWM, leftPWM);
    
    // Right motors - set direction and speed
    digitalWrite(RIGHT_MOTOR_DIR, rightSpeed >= 0 ? HIGH : LOW);
    softPwmWrite(RIGHT_MOTOR_PWM, rightPWM);
}

// Initialize Xbox controller
bool initController() {
    joystick_fd = open(JOYSTICK_DEVICE, O_RDONLY | O_NONBLOCK);
    if (joystick_fd < 0) {
        std::cerr << "Error: Could not open joystick device " << JOYSTICK_DEVICE << std::endl;
        std::cerr << "Make sure your Xbox controller is connected." << std::endl;
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
    initMotors();
    
    if (!initController()) {
        std::cerr << "Failed to initialize controller. Exiting." << std::endl;
        return 1;
    }
    
    std::cout << "\nRover ready!" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Right Trigger: Forward" << std::endl;
    std::cout << "  Left Trigger: Backward" << std::endl;
    std::cout << "  Left Stick (X-axis): Steering" << std::endl;
    std::cout << "  Press Ctrl+C to exit\n" << std::endl;
    
    // Controller state
    float rightTrigger = 0.0f;  // 0.0 to 1.0
    float leftTrigger = 0.0f;   // 0.0 to 1.0
    float steeringX = 0.0f;     // -1.0 to 1.0
    
    struct js_event event;
    
    // Main control loop
    while (running) {
        // Read controller events
        while (read(joystick_fd, &event, sizeof(event)) > 0) {
            // Process only axis events
            if (event.type == JS_EVENT_AXIS) {
                float value = event.value / 32767.0f; // Normalize to -1.0 to 1.0
                
                switch (event.number) {
                    case AXIS_LEFT_STICK_X:
                        steeringX = applyDeadzone(value, DEADZONE);
                        break;
                        
                    case AXIS_RIGHT_TRIGGER:
                        // Xbox triggers range from -1.0 (unpressed) to 1.0 (pressed)
                        rightTrigger = applyDeadzone((value + 1.0f) / 2.0f, DEADZONE);
                        break;
                        
                    case AXIS_LEFT_TRIGGER:
                        leftTrigger = applyDeadzone((value + 1.0f) / 2.0f, DEADZONE);
                        break;
                }
            }
        }
        
        // Calculate throttle (triggers control forward/backward)
        float throttle = rightTrigger - leftTrigger; // -1.0 to 1.0
        
        // Apply motor control with differential steering
        setMotorSpeed(throttle, steeringX);
        
        // Small delay to prevent CPU overload
        usleep(10000); // 10ms = 100Hz update rate
    }
    
    // Cleanup
    std::cout << "Stopping motors..." << std::endl;
    stopAllMotors();
    close(joystick_fd);
    std::cout << "Rover shutdown complete." << std::endl;
    
    return 0;
}