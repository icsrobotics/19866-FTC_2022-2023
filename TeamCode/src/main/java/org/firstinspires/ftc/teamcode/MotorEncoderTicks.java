package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Config
@TeleOp(name = "View Encoder Motor Positions", group = "Linear Opmode")
public class MotorEncoderTicks extends LinearOpMode{

    RobotHardware robot = new RobotHardware(this);
    MyPIDController DrivePIDControl = new MyPIDController(0.07, 0.05, 0);
    MyPIDController ArmPIDControl = new MyPIDController(0.05, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.setMsTransmissionInterval(50);

        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Arm Motor Position", robot.leftArmMotor.getCurrentPosition());
            telemetry.addData("Right Arm Motor Position", robot.rightArmMotor.getCurrentPosition());

            telemetry.addData("Back Left Motor", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Motor", robot.backRight.getCurrentPosition());
            telemetry.addData("Front Left Motor", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Motor", robot.frontRight.getCurrentPosition());

            telemetry.addData("Error Value For Drivetrain", DrivePIDControl.error);
            telemetry.addData("Error Value For Arm", ArmPIDControl.error);

            telemetry.update();
        }
    }
}

