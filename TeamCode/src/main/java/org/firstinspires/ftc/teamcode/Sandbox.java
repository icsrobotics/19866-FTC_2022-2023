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
            int targetPosition = 500;

            double blPower = DrivePIDControl.PIDControl(targetPosition, robot.backLeft.getCurrentPosition(), 50);
            double brPower = DrivePIDControl.PIDControl(targetPosition, robot.backRight.getCurrentPosition(), 50);
            double flPower = DrivePIDControl.PIDControl(targetPosition, robot.frontLeft.getCurrentPosition(), 50);
            double frPower = DrivePIDControl.PIDControl(targetPosition, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

           /* // Low: 1936, Med: 2770, High: 4075
            int lowTargetPosition = 1936;
            int midTargetPosition = 2770;
            int highTargetPosition = 4075;

            double leftArmPower = ArmPIDControl.PIDControl(lowTargetPosition, robot.leftArmMotor.getCurrentPosition());
            double rightArmPower = ArmPIDControl.PIDControl(lowTargetPosition, robot.rightArmMotor.getCurrentPosition());

            robot.leftArmMotor.setPower(leftArmPower);
            robot.rightArmMotor.setPower(rightArmPower);*/

            telemetry.addData("Left Arm Motor Position", robot.leftArmMotor.getCurrentPosition());
            telemetry.addData("Right Arm Motor Position", robot.rightArmMotor.getCurrentPosition());

            telemetry.addData("Back Left Motor", robot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Motor", robot.backRight.getCurrentPosition());
            telemetry.addData("Front Left Motor", robot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Motor", robot.frontRight.getCurrentPosition());

            telemetry.addData("Error Value", DrivePIDControl.error);

            telemetry.update();
        }
    }
}

