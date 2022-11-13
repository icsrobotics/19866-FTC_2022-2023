package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MyPIDController {
    // PiD Constants
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    private double lastError = 0;
    private double lastIntegral = 0;

    ElapsedTime timer = new ElapsedTime();
    double currentTime, elapsedTime, previousTime;

    public MyPIDController (double P, double I, double D) {
        this.Kp = P;
        this.Ki = I;
        this.Kd = D;
    }

    // refrence = setpoint you want
    // state = what the motor or servo is outputting usually acsessed by .getwhatever()
    public double PIDControl (double refrence, double state) {
        currentTime = timer.seconds();
        elapsedTime = (double)(currentTime - previousTime);

        double error = refrence - state;
        double derivative = (error - lastError) / elapsedTime;
        double integral =  (error + lastIntegral) * elapsedTime;

        // Setting all of the stuff after power or whatever applired
        lastError = error;
        lastIntegral = integral;
        previousTime = currentTime;

        double output = (Kp * error) + (Kd * derivative) + (Ki * integral);
        return output;
    }
}