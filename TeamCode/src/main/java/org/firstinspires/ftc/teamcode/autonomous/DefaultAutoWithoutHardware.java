package org.firstinspires.ftc.teamcode.autonomous;

// Refrenced code: https://github.com/cobalt-colts/AprilTag-Workshop/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auton/AprilTagAutonomousInitDetectionExample.java


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MyPIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "Other... Autonomous", group = "Linear Opmode")
public class DefaultAutoWithoutHardware extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    RobotHardware robot = new RobotHardware(this);
    MyPIDController DrivePIDController = new MyPIDController(0.07, 0.05, 0.01);

    // Drive Motors. Ports
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backRight = null;
    public DcMotorEx backLeft = null;

    static final double FEET_PER_METER = 3.28084;

    //Lens intrinsics: UNITS ARE PIXELS!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family. Ours is 6, 7, 8
    int LEFT = 6;
    int MIDDLE = 7;
    int RIGHT = 8;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        backLeft = (DcMotorEx) hardwareMap.dcMotor.get("Back_Left");
        backRight = (DcMotorEx) hardwareMap.dcMotor.get("Back_Right");
        frontRight = (DcMotorEx) hardwareMap.dcMotor.get("Front_Right");
        frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("Front_Left");

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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);}

            @Override public void onError(int errorCode) {
                telemetry.addData("Robot", "In Peril. God Bless");
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);

        //The INIT-loop: This REPLACES waitForStart!
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        // The START command just came in: now work off the latest snapshot acquired during the init loop
        /* Update the telemetry */
        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        init();
        /* Actually do something useful */

        // GAMEPLAN!!
        /*
         * 1. Forward
         * 2. Move to parking space
         *  a. Strafe left for #6 barcode / 1
         *  b. Stay still for #7 barcode / 2
         *  c. Strafe right for #8 barcode / 3
         * 3. Have fun
         * GOOD LUCK!
         */

        //Drivetrain values
        double oneTile = 1000;
        double errorRange = 10;

        double frPower = DrivePIDController.PIDControl(oneTile, frontRight.getCurrentPosition(), errorRange);
        double flPower = DrivePIDController.PIDControl(oneTile, frontLeft.getCurrentPosition(), errorRange);
        double brPower = DrivePIDController.PIDControl(oneTile, backRight.getCurrentPosition(), errorRange);
        double blPower = DrivePIDController.PIDControl(oneTile, backLeft.getCurrentPosition(), errorRange);

        if(tagOfInterest == null) {
            //default trajectory here if preferred
            // FORWARD
            frontRight.setPower(frPower);
            frontLeft.setPower(flPower);
            backRight.setPower(brPower);
            backLeft.setPower(blPower);

        } else if(tagOfInterest.id == LEFT) {
            //left trajectory - 1

            // FORWARD
            robot.frontRight.setPower(frPower);
            robot.frontLeft.setPower(flPower);
            robot.backRight.setPower(brPower);
            robot.backLeft.setPower(blPower);

            //LEFT
            frontRight.setPower(frPower);
            frontLeft.setPower(-flPower);
            backRight.setPower(-brPower);
            backLeft.setPower(blPower);

            telemetry.addData("Robot", "LEFT");
            telemetry.update();
        } else if(tagOfInterest.id == MIDDLE) {
            //middle trajectory - 2

            // FORWARD
            frontRight.setPower(frPower);
            frontLeft.setPower(flPower);
            backRight.setPower(brPower);
            backLeft.setPower(blPower);

            telemetry.addData("Robot", "MIDDLE");
            telemetry.update();
        } else {
            //right trajectory - 3

            // FORWARD
            frontRight.setPower(frPower);
            frontLeft.setPower(flPower);
            backRight.setPower(brPower);
            backLeft.setPower(blPower);

            //RIGHT
            frontRight.setPower(-frPower);
            frontLeft.setPower(flPower);
            backRight.setPower(brPower);
            backLeft.setPower(-blPower);

            telemetry.addData("Robot", "RIGHT");
            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}