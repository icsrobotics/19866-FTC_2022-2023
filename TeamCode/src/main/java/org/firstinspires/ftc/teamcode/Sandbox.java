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

//        endServo  = hardwareMap.get(Servo.class, "Servo");
        leftArmMotor = hardwareMap.get(DcMotor.class, "Left_Arm");
        rightArmMotor = hardwareMap.get(DcMotor.class, "Right_Arm");

        while (opModeIsActive()) {
            /*// Rising edge detector. Toggle for intake when Y is pressed
            // Using the toggle variable to control the robot.
            if (gamepad2.y && !intakeToggle) {
                if (endServo.getPosition() == 0.5) endServo.setPosition(0.6);
                else endServo.setPosition(0.5);

                intakeToggle = true;
            } else if (!gamepad2.y) intakeToggle = false;

            // Manual Optrion of intake. When b is pressed it moves all in. Used for testing methinks
            if (gamepad2.b) {
                endServo.setPosition(0.6);
            } else {
                endServo.setPosition(0.5);
            }*/
           /* int armLevel = 0;

            if (gamepad2.y){
                armLevel = armLevel + 1;
                telemetry.addData("Dpad Up", "Pressed");
                telemetry.addData("Arm Level Increased to:", armLevel);
                if (armLevel == 4) {
                    armLevel = 0;
                    telemetry.addLine("Toggle level resetted");
                }
            }
            switch (armLevel) {
                case 1:
                    double level1LeftPower = pidControl.PIDControl(10, leftArmMotor.getCurrentPosition());
                    double level1RightPower = pidControl.PIDControl(10, rightArmMotor.getCurrentPosition());
                    leftArmMotor.setPower(level1LeftPower);
                    rightArmMotor.setPower(level1LeftPower);
                    break;
                case 2:
                    double level2LeftPower = pidControl.PIDControl(15, leftArmMotor.getCurrentPosition());
                    double level2RightPower = pidControl.PIDControl(15, rightArmMotor.getCurrentPosition());
                    leftArmMotor.setPower(level2LeftPower);
                    rightArmMotor.setPower(level2LeftPower);
                    break;
                case 3:
                    double level3LeftPower = pidControl.PIDControl(20, leftArmMotor.getCurrentPosition());
                    double level3RightPower = pidControl.PIDControl(20, rightArmMotor.getCurrentPosition());
                    leftArmMotor.setPower(level3LeftPower);
                    rightArmMotor.setPower(level3LeftPower);
                    break;*/
                double power = -gamepad2.right_stick_y * robot.MAX_POWER;
                leftArmMotor.setPower(power);
                rightArmMotor.setPower(power);
        }


            /*telemetry.addData("Arm Position", leftArmMotor.getCurrentPosition());
            telemetry.addData("Arm Position", rightArmMotor.getCurrentPosition());*/

            telemetry.addData("Right stck", gamepad2.right_stick_y);
            telemetry.addData("Left stick", gamepad2.left_stick_y);

            telemetry.update();
    }
}
