package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Mecanum Driving Tank Drive", group = "Linear Opmode")
public class Mecanum_TankDrive extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    double left;
    double right;
    double strafe;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            left = -gamepad1.left_stick_y * RobotHardware.MAX_POWER;
            right = -gamepad1.right_stick_y * RobotHardware.MAX_POWER;
            strafe = -gamepad1.left_stick_x * RobotHardware.MAX_POWER;

            // Forwawrd and Backward
            robot.backLeft.setPower(left);
            robot.frontLeft.setPower(left);
            robot.backRight.setPower(right);
            robot.frontRight.setPower(right);

        }
    }
}
