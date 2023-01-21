package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotHardware;

/* Refrenced code: https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example*/

@Config
@TeleOp(name = "Field Centric Driving", group = "TeleOp")
public class Field_Centric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    double SOME_VARIABLE = 0.8;
    Orientation angles;

    @Override public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.setMsTransmissionInterval(50);

        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            final double globalAngle = angles.firstAngle;

            //Finds the hypotenous of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
            final double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //Finds the robot's angle from the raw values of the joystick
            final double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI /4;
            final double rightX = gamepad1.right_stick_x;

            double v1 = r * Math.sin(robotAngle - globalAngle/57) - rightX;
            double v2 = r * Math.cos(robotAngle - globalAngle/57) + rightX;
            double v3 = r * Math.cos(robotAngle - globalAngle/57) - rightX;
            double v4 = r * Math.sin(robotAngle - globalAngle/57) + rightX;

            if (Math.abs(v1) > 1 || Math.abs(v2) > 1 || Math.abs(v3) > 1 || Math.abs(v4) > 1 ) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(v1), Math.abs(v2));
                max = Math.max(Math.abs(v3), max);
                max = Math.max(Math.abs(v4), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                v1 /= max;
                v2 /= max;
                v3 /= max;
                v4 /= max;
            }

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

            robot.frontRight.setPower(v1);
            robot.frontLeft.setPower(v2);
            robot.backRight.setPower(v3);
            robot.backLeft.setPower(v4);

            telemetry.addData("Heading ", globalAngle);
            telemetry.addData("Stick ", robotAngle);
            telemetry.update();

            // OLD DRIVE CODE!
            /*            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.frontLeft.setPower(v1);
            robot.frontRight.setPower(v2);
            robot.backLeft.setPower(v3);
            robot.backRight.setPower(v4);*/

            // WHATS GOIN ON. Telemetry tells you
/*          telemetry.addData("Back Left Motor", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Motor", robot.backRight.getCurrentPosition());
            telemetry.addData("Front Left Motor", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Motor", robot.frontRight.getCurrentPosition());

            telemetry.addData("Left Arm Motor", robot.rightArmMotor.getCurrentPosition());
            telemetry.addData("Right Arm Motor", robot.leftArmMotor.getCurrentPosition());

            telemetry.addData("Servo Position", robot.endServo.getPosition());

            telemetry.update();*/
        }
    }
}


