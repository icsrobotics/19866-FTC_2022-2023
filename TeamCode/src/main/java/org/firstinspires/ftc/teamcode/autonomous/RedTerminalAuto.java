package org.firstinspires.ftc.teamcode.autonomous;// Refrenced code: https://github.com/cobalt-colts/AprilTag-Workshop/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auton/AprilTagAutonomousInitDetectionExample.java

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.MyPIDController;

import java.util.ArrayList;

@Config
@Autonomous(name = "Red Terminal Autonomous", group = "Linear Opmode")
public class RedTerminalAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    RobotHardware robot = new RobotHardware(this);

    MyPIDController DrivePIDController = new MyPIDController(0.07, 0.05, 0.03);
    MyPIDController ArmPIDController = new MyPIDController(0.05, 0, 0);

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

    AprilTagDetection tagOfInterest = null;

    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(camera, 10); //0 originally
        telemetry.setMsTransmissionInterval(50);

        robot.init();
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

         /* The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {
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
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");
                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

         /* The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */


        //Update the telemetry

        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //Actually do something useful

        // GENERAL:
            // suck in cone
            // right tile
            //forward tile
            // right 1/2 tile
            //raise arm
            //drop cone
            // drop arm
            //left 1/2 tile

        if(tagOfInterest == null || tagOfInterest.id == LEFT /*#1 */) {
            //two tiles left

            telemetry.addData("Robot", "LEFT OR NOT DETECTED");
            telemetry.update();
        } else if(tagOfInterest.id == MIDDLE /*#2*/){
            //one tiles left

            telemetry.addData("Robot", "MIDDLE");
            telemetry.update();

        } else if (tagOfInterest.id == RIGHT /*#3*/){
            // stay still

            telemetry.addData("Robot", "RIGHT");
            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

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

    void strafeLeft(){
        robot.frontRight.setPower(-oneTile);
        robot.frontLeft.setPower(oneTile);
        robot.backRight.setPower(-oneTile);
        robot.backLeft.setPower(oneTile);
    }

    void strafeRight(){
        robot.frontRight.setPower(oneTile);
        robot.frontLeft.setPower(-oneTile);
        robot.backRight.setPower(oneTile);
        robot.backLeft.setPower(-oneTile);
    }

    void moveForward(){
        robot.frontRight.setPower(oneTile);
        robot.frontLeft.setPower(oneTile);
        robot.backRight.setPower(oneTile);
        robot.backLeft.setPower(oneTile);
    }

    void moveBackward(){
        robot.frontRight.setPower(-oneTile);
        robot.frontLeft.setPower(-oneTile);
        robot.backRight.setPower(-oneTile);
        robot.backLeft.setPower(-oneTile);
    }
}
