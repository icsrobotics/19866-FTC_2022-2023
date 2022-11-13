package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware;

/* Refrenced code: https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example*/

@Config
@TeleOp(name = "Mecanum Driving POV", group = "Linear Opmode")
public class Mecanum_POV extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    double SOME_VARIABLE = 0.5;

    @Override public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.setMsTransmissionInterval(50);

        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            robot.frontLeft.setPower(v1 * SOME_VARIABLE);
            robot.frontRight.setPower(v2 * SOME_VARIABLE);
            robot.backLeft.setPower(v3 * SOME_VARIABLE);
            robot.backRight.setPower(v4 * SOME_VARIABLE);

            // ARM STUFF
            // Rising edge detector. Toggle for intake when Y is pressed
            // Using the toggle variable to control the robot.
            /*if (gamepad2.y && !robot.intakeToggle) {
                robot.endServo.setPosition(0.7);

                robot.intakeToggle = true;
            } else if (!gamepad2.y) robot.intakeToggle = false;*/
            boolean toggle = false;

            if (gamepad2.y) {
                toggle = !toggle;
            }
            if (toggle) {
                robot.endServo.setPosition(1.0);
            } else if (toggle == false) {
                robot.endServo.setPosition(0.5);
            }

            // Manual Optrion of intake. When b is pressed it moves all in. Used for testing methinks
            if (gamepad2.y) {
                robot.endServo.setPosition(1.0);
            } else if (gamepad2.a) {
                robot.endServo.setPosition(0);
            } else {
                robot.endServo.setPosition(0.5);
            }

            //ARM STUFF
            double power = -gamepad2.left_stick_y * robot.MAX_POWER;
            robot.leftArmMotor.setPower(power);
            robot.rightArmMotor.setPower(power);

            telemetry.addData("Left Arm Motor", robot.rightArmMotor.getPower());
            telemetry.addData("Right Arm Motor", robot.leftArmMotor.getPower());
            telemetry.addData("Servo Position", robot.endServo.getPosition());
            telemetry.update();
        }
    }
}


