/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @Date: 2019-09-12 14:51:36
 * @LastEditTime: 2019-10-11 16:39:57
 * @LastEditors: Please set LastEditors
 */
#include <Arduino.h>
#include "Pins.h"
/* #include "BalanceCar.h" */
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "Wire.h"
MPU6050 mpu;
KalmanFilter kalmanfilter;
const float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
const float balance_kp = 0.0, balance_kd = 0.0, balance_ki = 0.0;

void setup() {
    Serial.begin(115200);
    Serial.println("init");
    pinMode(AIN1, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(PWMA_LEFT, OUTPUT);
    pinMode(PWMB_RIGHT, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    Wire.begin();
    mpu.initialize();

    Serial.print("#");
    cli();
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0  = 0;
    /* Serial.print("#"); */
    TCCR0A = _BV(WGM01);                // CTC
    TCCR0B = _BV(CS02) | _BV(CS00);     // prescale 1024
    /* Serial.print("$"); */
    OCR0A = 255;        // 61.035 Hz
    /* Serial.print("%"); */
    TIMSK0 |= _BV(OCIE0A);
    sei();
    Serial.println("inited");
}

void loop() {
}

ISR(TIMER0_COMPA_vect) {
    static int encoder_right_pulse_num_speed = 0;
    static float last_angle = 0.0;
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
    const float kalmanfilter_angle = kalmanfilter.angle;
    const float balance_control_output =
        balance_kp * kalmanfilter_angle + balance_kd * kalmanfilter.Gyro_x + balance_ki * (kalmanfilter_angle - last_angle);
    last_angle = kalmanfilter_angle;

    const float pwm_wheel = constrain(balance_control_output, -255, 255);

    /* if (motion_mode != START && motion_mode != STOP && */
    /*     (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle)) { */
    /*     motion_mode = STOP; */
    /*     carStop(); */
    /* } */

    const bool back = pwm_wheel < 0;
    digitalWrite(AIN1, back);
    digitalWrite(BIN1, back);
    analogWrite(PWMA_LEFT, back ? -pwm_wheel : pwm_wheel);
    analogWrite(PWMB_RIGHT, back ? -pwm_wheel : pwm_wheel);
}
