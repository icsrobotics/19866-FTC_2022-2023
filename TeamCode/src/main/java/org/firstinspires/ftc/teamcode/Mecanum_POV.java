package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/* Refrenced code: https://github.com/DeltaRobotics-9351/SkyStone-Regional/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/MecanumWheels.java */

@Config
@TeleOp(name = "Mecanum Driving POV", group = "Linear Opmode")
public class Mecanum_POV extends LinearOpMode {

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;

    @Override public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        backLeft = hardwareMap.dcMotor.get("Back_Left");
        backRight = hardwareMap.dcMotor.get("Back_Right");
        frontRight = hardwareMap.dcMotor.get("Front_Right");
        frontLeft = hardwareMap.dcMotor.get("Front_Left");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        double drive;
        float turnleft;
        float turnright;
        double left = 0;
        double right = 0;
        double max;

        //wheel motor power
        double wheelFrontRightPower = 0;
        double wheelFrontLeftPower = 0;
        double wheelBackRightPower = 0;
        double wheelBackLeftPower = 0;

        double turbo = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            double y1 = gamepad1.left_stick_y;
            double x1 = -gamepad1.left_stick_x;
            double x2 = -gamepad1.right_stick_x;
            wheelFrontRightPower = y1 - x2 - x1;
            wheelBackRightPower = y1 - x2 + x1;
            wheelFrontLeftPower = y1 + x2 + x1;
            wheelBackLeftPower = y1 + x2 - x1;

            max = Math.max(Math.abs(wheelFrontRightPower), Math.max(Math.abs(wheelBackRightPower),
                    Math.max(Math.abs(wheelFrontLeftPower), Math.abs(wheelBackLeftPower))));

            if (max > 1.0)
            {
                wheelFrontRightPower /= max;
                wheelBackRightPower  /= max;
                wheelFrontLeftPower  /= max;
                wheelBackLeftPower   /= max;
            }

            wheelFrontRightPower *= turbo;
            wheelBackRightPower  *= turbo;
            wheelFrontLeftPower  *= turbo;
            wheelBackLeftPower   *= turbo;

            frontRight.setPower(wheelFrontRightPower);
            frontLeft.setPower(wheelFrontLeftPower);
            backRight.setPower(wheelBackRightPower);
            backLeft.setPower(wheelBackLeftPower);
        }
    }
}


