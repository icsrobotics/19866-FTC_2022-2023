package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(name = "POV Driving", group = "TeleOp")
public class POV extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    private final double driveAdjuster = 1;
    @Override public void runOpMode()  throws InterruptedException {
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.frontLeft.setPower(v1);
            robot.frontRight.setPower(v2);
            robot.backLeft.setPower(v3);
            robot.backRight.setPower(v4);

            // Servo STUFF
            if (gamepad2.y && !robot.intakeToggle) {
                if (robot.endServo.getPosition() == 0.5) robot.endServo.setPosition(1.0);
                else robot.endServo.setPosition(0.5);
                robot.intakeToggle = true;

            } else if (!gamepad2.y) robot.intakeToggle = false;

            // Optional servo stuff (debugging methinks?)
            if (gamepad2.b) robot.endServo.setPosition(1.0);
            if (gamepad2.a) robot.endServo.setPosition(0);
            //ARM STUFF
            double power = gamepad2.left_stick_y;
            robot.leftArmMotor.setPower(power);
            robot.rightArmMotor.setPower(power);

            // WHATS GOIN ON. Telemetry tells you
          telemetry.addData("Back Left Motor", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Motor", robot.backRight.getCurrentPosition());
            telemetry.addData("Front Left Motor", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Motor", robot.frontRight.getCurrentPosition());

            telemetry.addData("Left Arm Motor", robot.rightArmMotor.getCurrentPosition());
            telemetry.addData("Right Arm Motor", robot.leftArmMotor.getCurrentPosition());

            telemetry.addData("Servo Position", robot.endServo.getPosition());

            telemetry.update();

        }
    }
}
