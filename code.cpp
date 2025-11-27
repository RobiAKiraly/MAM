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
#define CONTROL_LOOP_DELAY 10000 // 10ms = 100Hz
 
// Xbox Controller Axis Mapping
#define AXIS_LEFT_STICK_X 0
#define AXIS_LEFT_TRIGGER 2
#define AXIS_RIGHT_TRIGGER 5
volatile sig_atomic_t running = 1;
void signalHandler(int signum) {
    std::cout << "\nShutting down rover safely..." << std::endl;
    running = 0;
}
 
class MotorController {
private:
    int leftENA;
    int leftIN1;
    int leftIN2;
    int rightENB;
    int rightIN3;
    int rightIN4;
    bool initialized;
public:
    MotorController(int lENA, int lIN1, int lIN2, int rENB, int rIN3, int rIN4)
        : leftENA(lENA), leftIN1(lIN1), leftIN2(lIN2),
          rightENB(rENB), rightIN3(rIN3), rightIN4(rIN4),
          initialized(false) {}
    bool initialize() {
        if (gpioInitialise() < 0) {
            std::cerr << "Failed to initialize pigpio library" << std::endl;
            return false;
        }
        gpioSetMode(leftENA, PI_OUTPUT);
        gpioSetMode(leftIN1, PI_OUTPUT);
        gpioSetMode(leftIN2, PI_OUTPUT);
        gpioSetMode(rightENB, PI_OUTPUT);
        gpioSetMode(rightIN3, PI_OUTPUT);
        gpioSetMode(rightIN4, PI_OUTPUT);
 
        gpioSetPWMfrequency(leftENA, PWM_FREQUENCY);
        gpioSetPWMfrequency(rightENB, PWM_FREQUENCY);
        gpioSetPWMrange(leftENA, PWM_RANGE);
        gpioSetPWMrange(rightENB, PWM_RANGE);
 
        gpioPWM(leftENA, 0);
        gpioPWM(rightENB, 0);

        gpioWrite(leftIN1, 0);
        gpioWrite(leftIN2, 0);
        gpioWrite(rightIN3, 0);
        gpioWrite(rightIN4, 0);
 
        initialized = true;
        std::cout << "Motors initialized on GPIO pins" << std::endl;
        return true;
    }
    void setSpeed(float throttle, float steering) {
        if (!initialized) return;
 
        throttle = std::clamp(throttle, -1.0f, 1.0f);
        steering = std::clamp(steering, -1.0f, 1.0f);
        float leftSpeed, rightSpeed;
 
        if (steering < 0) {
            leftSpeed = throttle * (1.0f + steering * TURN_AGGRESSION);
            rightSpeed = throttle;
        } else if (steering > 0) {
            leftSpeed = throttle;
            rightSpeed = throttle * (1.0f - steering * TURN_AGGRESSION);
        } else {
            leftSpeed = throttle;
            rightSpeed = throttle;
        }
 
        leftSpeed = std::clamp(leftSpeed, -1.0f, 1.0f);
        rightSpeed = std::clamp(rightSpeed, -1.0f, 1.0f);
 
        int leftPWM = static_cast<int>(std::abs(leftSpeed) * PWM_RANGE);
        int rightPWM = static_cast<int>(std::abs(rightSpeed) * PWM_RANGE);
 
        gpioWrite(leftIN1, leftSpeed >= 0 ? 1 : 0);
        gpioWrite(leftIN2, leftSpeed >= 0 ? 0 : 1);

        gpioPWM(leftENA, leftPWM);
        gpioWrite(rightIN3, rightSpeed >= 0 ? 1 : 0);
        gpioWrite(rightIN4, rightSpeed >= 0 ? 0 : 1);
        gpioPWM(rightENB, rightPWM);
    }
 
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
 

class XboxController {
private:
    int fileDescriptor;
    float rightTrigger;
    float leftTrigger;
    float steeringX;
    float deadzone;
    float applyDeadzone(float value) {
        if (std::abs(value) < deadzone)
            return 0.0f;
        float sign = value > 0 ? 1.0f : -1.0f;
        return sign * ((std::abs(value) - deadzone) / (1.0f - deadzone));
    }
 
public:
    XboxController(float deadzoneThreshold = DEADZONE)
        : fileDescriptor(-1), rightTrigger(0.0f), leftTrigger(0.0f),
          steeringX(0.0f), deadzone(deadzoneThreshold) {}
    ~XboxController() {
        if (fileDescriptor >= 0)
            close(fileDescriptor);
    }
 
    bool initialize(const char* device = JOYSTICK_DEVICE) {
        fileDescriptor = open(device, O_RDONLY | O_NONBLOCK);
        if (fileDescriptor < 0) {
            std::cerr << "Error: Could not open joystick device " << device << std::endl;
            return false;
        }

        std::cout << "Xbox controller connected successfully" << std::endl;
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
                    rightTrigger = applyDeadzone((value + 1.0f) / 2.0f);
                    break;
                case AXIS_LEFT_TRIGGER:
                    leftTrigger = applyDeadzone((value + 1.0f) / 2.0f);
                    break;
                }
            }
        }
    }
 
    float getThrottle() const { return rightTrigger - leftTrigger; }
    float getSteering() const { return steeringX; }
};

class RoverSystem {
public:
    std::unique_ptr<MotorController> motors;
    std::unique_ptr<XboxController> controller;
 
    RoverSystem(int lENA, int lIN1, int lIN2, int rENB, int rIN3, int rIN4) {
        motors = std::make_unique<MotorController>(lENA, lIN1, lIN2,
                                                   rENB, rIN3, rIN4);
        controller = std::make_unique<XboxController>();
    }
 
    bool initialize() {
        std::cout << "=== Mars Rover Control System ===\n";
        if (!motors->initialize()) return false;
        if (!controller->initialize()) return false;
        std::cout << "Rover ready!\n";
        return true;
    }
 
    void run() {
        while (running) {
            controller->update();
            motors->setSpeed(controller->getThrottle(),
                             controller->getSteering());
            usleep(CONTROL_LOOP_DELAY);
        }
        motors->stop();
    }
};
 
const int LEFT_ENA = 22;
const int LEFT_IN1 = 17;
const int LEFT_IN2 = 27;
const int RIGHT_ENB = 25;
const int RIGHT_IN3 = 23;
const int RIGHT_IN4 = 24;
 
int main() {
    signal(SIGINT, signalHandler);
    RoverSystem rover(LEFT_ENA, LEFT_IN1, LEFT_IN2,
                      RIGHT_ENB, RIGHT_IN3, RIGHT_IN4);
    if (!rover.initialize())
        return 1;
    rover.run();
    return 0;
}