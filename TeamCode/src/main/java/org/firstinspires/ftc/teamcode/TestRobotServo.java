package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Disabled
@Config
@TeleOp(name = "Servo Test", group = "Linear Opmode")
public class TestRobotServo extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it). wtf is this for?
            if (gamepad1.y) robot.endServo.setPosition(0.7);
            else if (gamepad1.a) robot.endServo.setPosition(-0.7);
            else {
                robot.endServo.setPosition(0.7);
            }

            // toggle servo. probably useful resource: https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
            boolean toggle = false;
            // Rising edge detector
            if (gamepad2.a) toggle = !toggle;

            if (toggle) robot.endServo.setPosition(1);
            else robot.endServo.setPosition(0.5);
            // Using the toggle variable to control the robot.

        }
    }
}
