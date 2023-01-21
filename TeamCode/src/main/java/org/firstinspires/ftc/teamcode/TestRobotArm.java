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

        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // reset encoder counts kept by motors.
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run forward for 5000 encoder counts.
        leftArmMotor.setTargetPosition(1000);
        rightArmMotor.setTargetPosition(1000);

        // set motors to run to target encoder position and stop with brakes on.
        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power. Movement will start. Sign of power is
        // ignored as sign of target encoder position controls direction when
        // running to position.

        leftArmMotor.setPower(0.25);
        rightArmMotor.setPower(0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && leftArmMotor.isBusy())   //leftArmMotor.getCurrentPosition() < leftArmMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", leftArmMotor.getCurrentPosition() + "  busy=" + leftArmMotor.isBusy());
            telemetry.addData("encoder-fwd-right", rightArmMotor.getCurrentPosition() + "  busy=" + rightArmMotor.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftArmMotor.setPower(0.0);
        rightArmMotor.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.

        sleep(5000);

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", leftArmMotor.getCurrentPosition());
            telemetry.addData("encoder-fwd-right-end", rightArmMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // From current position back up to starting point. In this example instead of
        // having the motor monitor the encoder we will monitor the encoder ourselves.

        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArmMotor.setTargetPosition(0);
        rightArmMotor.setTargetPosition(0);

        // Power sign matters again as we are running without encoder.
        leftArmMotor.setPower(-0.25);
        rightArmMotor.setPower(-0.25);

        while (opModeIsActive() && leftArmMotor.getCurrentPosition() > leftArmMotor.getTargetPosition())
        {
            telemetry.addData("encoder-back-left", leftArmMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right", rightArmMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        leftArmMotor.setPower(0.0);
        rightArmMotor.setPower(0.0);

        sleep(5000);

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power. Movement will start. Sign of power is
        // ignored as sign of target encoder position controls direction when
        // running to position.

        leftArmMotor.setPower(0.25);
        rightArmMotor.setPower(0.25);

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
