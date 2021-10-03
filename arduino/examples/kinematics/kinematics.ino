//wheel radius, r = 160mm = 0.16m
//wheel separation, l = 498mm = 0.498m

#include "Kinematics.h"

#define MOTOR_MAX_RPM 2400        // motor's maximum rpm
#define WHEEL_DIAMETER 0.32      // robot's wheel diameter expressed in meters
#define LR_WHEEL_DISTANCE 0.498   // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)

Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, LR_WHEEL_DISTANCE, PWM_BITS);

void setup() 
{
    Serial.begin(9600);
}

void loop() 
{
    Kinematics::output pwm;

    //simulated required velocities
    float linear_vel_x = 0.2;  // 1 m/s
    float angular_vel_z = 0; // 1 m/s

    //given the required velocities for the robot, you can calculate the rpm required for each motor
    pwm = kinematics.getPWM(linear_vel_x, angular_vel_z);

    Serial.print(" PWM REQUIRED LEFT MOTOR: ");
    // Assuming you have an encoder for each wheel, you can pass this RPM value to a PID controller 
    // as a setpoint and your encoder data as a feedback.
    Serial.print(pwm.motor1);

    Serial.print(" PWM REQUIRED RIGHT MOTOR: ");
    Serial.print(pwm.motor2);

    delay(5000);


    // This is a simulated feedback from each motor. We'll just pass the calculated rpm above for demo's sake.
    // In a live robot, these should be replaced with real RPM values derived from encoder.
    int motor1_feedback = pwm.motor1; //in rpm
    int motor2_feedback = pwm.motor2; //in rpm

    Kinematics::velocities vel;

    // Now given the RPM from each wheel, you can calculate the linear and angular velocity of the robot.
    // This is useful if you want to create an odometry data (dead reckoning)
    vel = kinematics.getVelocities(motor1_feedback, motor2_feedback);
    Serial.print(" VEL X: ");
    Serial.print(vel.linear_x, 4);

    Serial.print(" ANGULAR_Z: ");
    Serial.println(vel.angular_z, 4);
    Serial.println("");
}
