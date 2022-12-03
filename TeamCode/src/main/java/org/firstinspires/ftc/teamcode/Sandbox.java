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
    MyPIDController DrivePIDControl = new MyPIDController(0.05, 0, 0);
    MyPIDController ArmPIDControl = new MyPIDController(0.05, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.setMsTransmissionInterval(50);

        robot.init();
        waitForStart();

        while (opModeIsActive()) {
            int targetPosition = 2000;

            double blPower = DrivePIDControl.PIDControl(targetPosition, robot.backLeft.getCurrentPosition());
            double brPower = DrivePIDControl.PIDControl(targetPosition, robot.backRight.getCurrentPosition());
            double flPower = DrivePIDControl.PIDControl(targetPosition, robot.frontLeft.getCurrentPosition());
            double frPower = DrivePIDControl.PIDControl(targetPosition, robot.frontRight.getCurrentPosition());

            // Low: 1936, Med: 2770, High: 4075
            int lowTargetPosition = 1936;
            int midTargetPosition = 2770;
            int highTargetPosition = 4075;

            double leftArmPower = ArmPIDControl.PIDControl(lowTargetPosition, robot.leftArmMotor.getCurrentPosition());
            double rightArmPower = ArmPIDControl.PIDControl(lowTargetPosition, robot.rightArmMotor.getCurrentPosition());

            robot.leftArmMotor.setPower(leftArmPower);
            robot.rightArmMotor.setPower(rightArmPower);

            telemetry.addData("Left Arm Motor Position", robot.leftArmMotor.getCurrentPosition());
            telemetry.addData("Right Arm Motor Position", robot.rightArmMotor.getCurrentPosition());
            telemetry.addData("Kp", DrivePIDControl.Kp);
            telemetry.addData("Ki", DrivePIDControl.Ki);
            telemetry.addData("Kd", DrivePIDControl.Kd);

            telemetry.update();
        }
    }
}

