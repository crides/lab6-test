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
const float dt = 0.05, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
const float balance_kp = 14.0, balance_kd = 0.0, balance_ki = 0.0;

void setup() {
    /* Serial.begin(115200); */
    /* Serial.println("init"); */
    pinMode(AIN1, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(PWMA_LEFT, OUTPUT);
    pinMode(PWMB_RIGHT, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);
    Wire.begin();
    mpu.initialize();

    // set compare match register for 1kHz increments
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);      // prescale 64x + CTC
    TCNT1  = 0;
    OCR1A = 16000000 / (64 * 1000) - 1;
    TIFR1 |= _BV(OCF1A);
    TIMSK1 |= _BV(OCIE1A);
    sei();
    /* Serial.println("inited"); */
}

static void balance() {
    static int encoder_right_pulse_num_speed = 0;
    static float last_angle = 0.0;
    static uint8_t cycle_count = 0;
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    kalmanfilter.Angle(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

    if (cycle_count < 20) {
        cycle_count ++;
        return;
    }
    cycle_count = 0;

    const float kalmanfilter_angle = kalmanfilter.angle + 1.42;
    const float balance_control_output =
        balance_kp * kalmanfilter_angle + balance_kd * kalmanfilter.Gyro_x + balance_ki * (kalmanfilter_angle - last_angle);
    last_angle = kalmanfilter_angle;

    const float pwm_wheel = constrain(balance_control_output, -255, 255);

    if (kalmanfilter_angle < -20 || 20 < kalmanfilter_angle) {
        digitalWrite(STBY_PIN, LOW);
        return;
    } else {
        digitalWrite(STBY_PIN, HIGH);
    }

    const bool back = pwm_wheel < 0;
    digitalWrite(AIN1, back);
    digitalWrite(BIN1, back);
    analogWrite(PWMA_LEFT, back ? -pwm_wheel : pwm_wheel);
    analogWrite(PWMB_RIGHT, back ? -pwm_wheel : pwm_wheel);
}

static volatile bool do_balance = false;

void loop() {
    if (do_balance) {
        balance();
        do_balance = false;
    }
}

ISR(TIMER1_COMPA_vect) {
    TIFR1 |= _BV(OCF1A);
    do_balance = true;
}
