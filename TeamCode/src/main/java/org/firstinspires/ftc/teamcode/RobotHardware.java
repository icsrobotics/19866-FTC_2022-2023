package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    /*public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/
    public static final double MAX_POWER = 0.7;

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
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        backLeft = myOpMode.hardwareMap.dcMotor.get("Back_Left");
        backRight = myOpMode.hardwareMap.dcMotor.get("Back_Right");
        frontRight = myOpMode.hardwareMap.dcMotor.get("Front_Right");
        frontLeft = myOpMode.hardwareMap.dcMotor.get("Front_Left");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
