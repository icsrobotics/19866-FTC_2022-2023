package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Drive Motors. Ports
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backRight = null;
    public DcMotorEx backLeft = null;

    // Arm Motors
    public DcMotor leftArmMotor = null;
    public DcMotor rightArmMotor = null;

    public Servo endServo = null;

    BNO055IMU imu;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public boolean intakeToggle = false;
    // encoder ticks to move ONE tile forward. EXACTLY
    int oneTile = 0;
    //encoder ticks to lift arm to HIGHEST level. 7465
    int highTargetPosition = 0;

    // Drivetrain power values
    double blPower;
    double brPower;
    double flPower;
    double frPower;

    // Arm power values
    double leftArmPower;
    double rightArmPower;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        backLeft = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("Back_Left");
        backRight = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("Back_Right");
        frontRight = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("Front_Right");
        frontLeft = (DcMotorEx) myOpMode.hardwareMap.dcMotor.get("Front_Left");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // ARM STUFF
        leftArmMotor = myOpMode.hardwareMap.dcMotor.get("Left_Arm");
        rightArmMotor = myOpMode.hardwareMap.dcMotor.get("Right_Arm");

        leftArmMotor.setDirection(DcMotor.Direction.REVERSE);
        rightArmMotor.setDirection(DcMotor.Direction.FORWARD);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        //SERVO STUFF
        endServo = myOpMode.hardwareMap.servo.get("End_Servo");

        //IMU Stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void strafeLeft(){
        frontRight.setPower(-oneTile);
        frontLeft.setPower(oneTile);
        backRight.setPower(-oneTile);
        backLeft.setPower(oneTile);
    }

    public void strafeRight(){
        frontRight.setPower(oneTile);
        frontLeft.setPower(-oneTile);
        backRight.setPower(oneTile);
        backLeft.setPower(-oneTile);
    }

    public void moveForward(){
        frontRight.setPower(oneTile);
        frontLeft.setPower(oneTile);
        backRight.setPower(oneTile);
        backLeft.setPower(oneTile);
    }

    public void moveBackward(){
        frontRight.setPower(-oneTile);
        frontLeft.setPower(-oneTile);
        backRight.setPower(-oneTile);
        backLeft.setPower(-oneTile);
    }
}
