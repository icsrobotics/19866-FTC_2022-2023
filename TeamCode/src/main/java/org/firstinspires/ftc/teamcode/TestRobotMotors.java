package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Disabled
@Config
@TeleOp(name = "Motor Test", group = "Linear Opmode")
public class TestRobotMotors extends LinearOpMode {

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;

    // reduces max speed to 0.7, reduces by 70 percent
    final double POWER = 0.7;

    public void runOpMode() {
        waitForStart();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        backLeft = hardwareMap.dcMotor.get("Back_Left");
        backRight = hardwareMap.dcMotor.get("Back_Right");
        frontRight = hardwareMap.dcMotor.get("Front_Right");
        frontLeft = hardwareMap.dcMotor.get("Front_Left");

        // This is accurate to 0 - Front_Left, 1 - Back_Left, 2 - Back_Right, 3 - Front_Right
        // Y - Back Left, B - Back Right, A - Front Left, X - Front Right
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection((DcMotorSimple.Direction.REVERSE));

        while (opModeIsActive()) {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            if (gamepad1.y){
                backLeft.setPower(POWER);
            } else if (gamepad1.b){
                backRight.setPower(POWER);
            } else if (gamepad1.a){
                frontLeft.setPower(POWER);
            } else if (gamepad1.x){
                frontRight.setPower(POWER);
            } else {
                frontLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
            }
        }
    }


}
