package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Mecanum Driving Tank Drive", group = "Linear Opmode")
public class Mecanum_TankDrive extends LinearOpMode {

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;

    double left;
    double right;

    // reduces max speed to 0.7, reduces by 70 percent
    final double MAX_POWER = 0.7;

    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        backLeft = hardwareMap.dcMotor.get("Back_Left");
        backRight = hardwareMap.dcMotor.get("Back_Right");
        frontRight = hardwareMap.dcMotor.get("Front_Right");
        frontLeft = hardwareMap.dcMotor.get("Front_Left");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            left = -gamepad1.left_stick_y * MAX_POWER;
            right = -gamepad1.right_stick_y * MAX_POWER;

            backLeft.setPower(left);
            frontLeft.setPower(left);
            backRight.setPower(right);
            frontRight.setPower(right);
        }
    }


}
