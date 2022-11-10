package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MyPIDController {

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0.3;

    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    public double PIDControl (double refrence, double state) {
        double error = refrence - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (refrence * Kf);
        return output;
    }
}