package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.MyPIDController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Sandbox (The little playground)", group = "Linear Opmode")
public class Sandbox extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    MyPIDController pidControl = new MyPIDController(0.05, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        int level = 0;

        telemetry.setMsTransmissionInterval(50);

//        robot.init();
        waitForStart();

        while (opModeIsActive()) {
/*
            int targetPosition = 100;

            double blPower = pidControl.PIDControl(targetPosition, robot.backLeft.getCurrentPosition());
            double brPower = pidControl.PIDControl(targetPosition, robot.backRight.getCurrentPosition());
            double flPower = pidControl.PIDControl(targetPosition, robot.frontLeft.getCurrentPosition());
            double frPower = pidControl.PIDControl(targetPosition, robot.frontRight.getCurrentPosition());

*/
            if (gamepad2.dpad_up) {
                telemetry.addData("Dpad Up", "Pressed");
                level = level + 1;
                telemetry.addData("Level", level);

                telemetry.update();
            }
        }
    }
}
