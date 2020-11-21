#include  "Position_estimation.h"
#include "Encoders.h"

Encoder RomiEncoders;
float x = 0;
float y = 0;
float theta = 0;
float time_interval = 50.0 / 1000.0; // s
unsigned long time_prev = millis();
unsigned long time_now = 0;

void Position::Init(void)
{
    time_prev = millis();
    x = 0;
    y = 0;
    theta = 0;
}

void Position::Stop(void)
{
    time_prev = millis();
    x = 0; 
    y = 0;
    theta = 0;
}

Position::pose_data Position::ReadPose(void)
{
    return {x,y,theta};
}

void Position::PrintPose(void)
{
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(theta);
}

void Position::UpdatePose(float target_speed_left, float target_speed_right)
{
    time_now = millis();
    if(time_now - time_prev >= 50) //update every 50ms for practical reasons
    {
        time_prev = time_now;
        float speed_left = RomiEncoders.ReadVelocityLeft(); // mm/s
        float speed_right = RomiEncoders.ReadVelocityRight(); // mm/s

        if (target_speed_left == target_speed_right || speed_left == speed_right) {
            handleStraight(speed_left, speed_right);
        } else {
            handleCurved(speed_left, speed_right);
        }
        PrintPose();
    }
}

void Position::handleStraight(float speed_left, float speed_right) {
    float V = (speed_left + speed_right) / 2.0; // mm/s

    x = x + V * cos(theta) * time_interval; // mm
    y = y + V * sin(theta) * time_interval; // mm
    theta = theta; // rad
}

void Position::handleCurved(float speed_left, float speed_right) {

    float R = (l/2.0) * (speed_right + speed_left) / (speed_right - speed_left); // mm
    float w = (speed_right - speed_left) / l; //  rad/s

    x = x - R * sin(theta) + R * sin(theta + w * time_interval); // mm
    y = y + R * cos(theta) - R * cos(theta + w * time_interval); // mm
    theta = theta + w * time_interval; // rad/s
}

