package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class SandboxDetectPosition {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // PUT CAMERA IN USB 2.0 PORT!!!
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS. maybe we can measure this but later on just to tune it :)
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family. CHANGED TO OUR TAGS
    int LEFT = 6;
    int MIDDLE = 7;
    int RIGHT = 8;

    public int parkZone = 0;

    AprilTagDetection tagOfInterest = null;
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public SandboxDetectPosition (LinearOpMode opmode) {
        myOpMode = opmode;
    }


    public void init() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0) {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }
            }

        } else {
            telemetry.addLine("Don't see tag of interest :(");
            if(tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
            }

        }

        telemetry.update();

        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            parkZone = 1;

            telemetry.addData("Robot", "LEFT OR NOT DETECTED");
            telemetry.update();
        } else if(tagOfInterest.id == MIDDLE){
            parkZone = 2;

            telemetry.addData("Robot", "MIDDLE");
            telemetry.update();
        } else if (tagOfInterest.id == RIGHT){
            parkZone = 3;

            telemetry.addData("Robot", "RIGHT");
            telemetry.update();
        }
    }
}
