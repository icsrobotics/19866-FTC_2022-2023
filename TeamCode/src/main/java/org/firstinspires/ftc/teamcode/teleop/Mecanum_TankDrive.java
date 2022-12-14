package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Disabled
@Config
@TeleOp(name = "Mecanum Driving Tank Drive", group = "Linear Opmode")
public class Mecanum_TankDrive extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    double left;
    double right;
    double strafe;
    double MAX_POWER = 1.0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            left = -gamepad1.left_stick_y * MAX_POWER;
            right = -gamepad1.right_stick_y * MAX_POWER;
            strafe = -gamepad1.left_stick_x * MAX_POWER;

            // Forwawrd and Backward
            robot.backLeft.setPower(left);
            robot.frontLeft.setPower(left);
            robot.backRight.setPower(right);
            robot.frontRight.setPower(right);
        }
    }
}
