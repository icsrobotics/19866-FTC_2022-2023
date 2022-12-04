package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MyPIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;

/* Refrenced code: https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example*/

@Config
@Autonomous(name = "Default Autonomous", group = "Linear Opmode")
public class DefaultAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    MyPIDController DrivePIDController = new MyPIDController(0.05, 0, 0);
    double SOME_VARIABLE = 0.8;
    double blPower;
    double brPower;
    double flPower;
    double frPower;

    double oneTile = 1000;

    @Override public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.setMsTransmissionInterval(50);

        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.init();

        waitForStart();

        while (opModeIsActive()) {

            // move forward one tile
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(-oneTile, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(-oneTile, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);
        }
    }
}


