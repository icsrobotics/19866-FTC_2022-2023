package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MyPIDController {
    // PiD Constants
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    public double error;

    private double lastError = 0;
    private double lastIntegral = 0;

    ElapsedTime timer = new ElapsedTime();
    double currentTime = timer.seconds();;

    public MyPIDController (double P, double I, double D) {
        this.Kp = P;
        this.Ki = I;
        this.Kd = D;
    }

    // refrence = setpoint you want
    // state = what the motor or servo is outputting usually acsessed by .getwhatever()
    public double PIDControl (double reference, double state, double error_range) {
        error = reference - state;
        double derivative = (error - lastError) / currentTime;
        double integral =  (error + lastIntegral) * currentTime;

        // Setting all of the stuff after power or whatever applied
        lastError = error;
        lastIntegral = integral;

        double output = (Kp * error) + (Kd * derivative) + (Ki * integral);
        if (error <= error_range) output = 0;

        return output;
    }
}