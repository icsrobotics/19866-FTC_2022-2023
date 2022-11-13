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
    MyPIDController pidControl = new MyPIDController();

    public Servo endServo = null;
    public DcMotor leftArmMotor = null;
    public DcMotor rightArmMotor = null;

    boolean intakeToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        endServo  = hardwareMap.get(Servo.class, "Servo");

        while (opModeIsActive()) {
           if (gamepad2.dpad_up) {
               telemetry.addData("Dpad Up", "Pressed");
               telemetry.update();
           }

            boolean toggle = false;

            if (gamepad2.y && !toggle) {
                toggle = !toggle;
            }
            if (toggle) {
                robot.endServo.setPosition(1.0);
            } else if (toggle == false) {
                robot.endServo.setPosition(0.5);
            }

        }

            telemetry.addData("Servo Position", endServo.getPosition());
            telemetry.update();
    }
}
