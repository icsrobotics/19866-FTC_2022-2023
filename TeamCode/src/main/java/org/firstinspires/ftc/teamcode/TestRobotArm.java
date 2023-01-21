package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@TeleOp(name = "Arm Test", group = "Linear Opmode")
public class TestRobotArm extends LinearOpMode {

    DcMotor leftArmMotor = null;
    DcMotor rightArmMotor = null;
    double position = 100;

    public void runOpMode() {
        waitForStart();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        MyPIDController pidController = new MyPIDController(0.07, 0, 0);

        leftArmMotor = hardwareMap.dcMotor.get("Left_Arm");
        rightArmMotor = hardwareMap.dcMotor.get("Right_Arm");


        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            double leftPower = pidController.PIDControl(position, leftArmMotor.getCurrentPosition(), 10);
            double rightPower = pidController.PIDControl(position, rightArmMotor.getCurrentPosition(), 10);

            leftArmMotor.setPower(leftPower);
            rightArmMotor.setPower(rightPower);

            telemetry.addData("Left Arm Position: ", leftArmMotor.getCurrentPosition());
            telemetry.addData("Right Arm Position: ", rightArmMotor.getCurrentPosition());

            telemetry.addData("Left Arm Power: ", leftPower);
            telemetry.addData("Right Arm Power: ", rightPower);

            telemetry.update();

            if (gamepad1.b) {
                leftArmMotor.setPower(0);
                rightArmMotor.setPower(0);
            }
        }
    }
}
