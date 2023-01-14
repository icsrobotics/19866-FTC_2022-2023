package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@Autonomous(name = "Move Forward", group = "Linear Opmode")
public class Sandbox extends LinearOpMode {
    int oneTile = 1075;

    // Drivetrain power values
    double blPower;
    double brPower;
    double flPower;
    double frPower;

    @Override public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        MyPIDController DrivePIDController = new MyPIDController(0.05, 0, 0);
        RobotHardware robot = new RobotHardware(this);

        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 5);
            flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 5);
            brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 5);
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 5);

            robot.frontRight.setPower(-frPower);
            robot.frontLeft.setPower(-flPower);
            robot.backRight.setPower(-brPower);
            robot.backLeft.setPower(-blPower);


            telemetry.addData("Back Left Motor", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Motor", robot.backRight.getCurrentPosition());
            telemetry.addData("Front Left Motor", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Motor", robot.frontRight.getCurrentPosition());
            telemetry.update();
        }
    }
}


