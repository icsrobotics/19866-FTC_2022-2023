package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Test", group = "Linear Opmode")
public class TestRobotServo extends LinearOpMode {

    public Servo servo = null;

    // change servo position to 0.7
    final double POSITION = 0.7;

    public void runOpMode() {
        waitForStart();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        servo = hardwareMap.servo.get("Servo");

        while (opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            if (gamepad1.y) servo.setPosition(POSITION);
            else if (gamepad1.a) servo.setPosition(-POSITION);
            else {
                servo.setPosition(POSITION);
            }

            // toggle servo. probably useful resource: https://gm0.org/en/latest/docs/software/tutorials/gamepad.html
            boolean toggle = false;
            // Rising edge detector
            if (gamepad2.a) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                toggle = !toggle;
            }
            // Using the toggle variable to control the robot.
            if (toggle) {
                servo.setPosition(1);
            }
            else {
                servo.setPosition(0.5);
            }
        }
    }
}
