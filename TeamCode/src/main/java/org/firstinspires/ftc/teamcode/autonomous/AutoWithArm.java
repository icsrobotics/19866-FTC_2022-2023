package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MyPIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Config
@Autonomous(name = "Autonomous With Arm", group = "Linear Opmode")
public class AutoWithArm extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 6;
    int MIDDLE = 7;
    int RIGHT = 8;

    AprilTagDetection tagOfInterest = null;

    //Arm Motors and Servo
    // ARM STUFF
    DcMotor leftArm = null;
    DcMotor rightArm = null;

    Servo endServo = null;


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.setMsTransmissionInterval(50);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT); }
            @Override public void onError(int errorCode) {/* God help you if there is an error here. */}
        });

        //Roadrunner stuff
        double oneTile = 18; // 18 in for one tile
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startpose = new Pose2d(0, 0, 0);

        //Robot Hardware
        leftArm = hardwareMap.dcMotor.get("Left_Arm");
        rightArm = hardwareMap.dcMotor.get("Right_Arm");

        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        endServo = hardwareMap.servo.get("End_Servo");

        // PID Controller
        MyPIDController pidController = new MyPIDController(0.07, 0, 0);
        int position = 2450;
        int errorRange = 10;

        double leftPower = pidController.PIDControl(position, leftArm.getCurrentPosition(), errorRange);
        double rightPower = pidController.PIDControl(position, rightArm.getCurrentPosition(), errorRange);

        drive.setPoseEstimate(startpose);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startpose)
                .forward(oneTile)
                .addDisplacementMarker(() -> {
                    // Run arm here!
                    leftArm.setPower(leftPower);
                    rightArm.setPower(rightPower);
                })
                .waitSeconds(1)
                .strafeLeft(oneTile)
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startpose)
                .forward(oneTile)
                .addDisplacementMarker(() -> {
                    // Run arm here!
                    leftArm.setPower(leftPower);
                    rightArm.setPower(rightPower);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startpose)
                .forward(oneTile)
                .addDisplacementMarker(() -> {
                    // Run arm here!
                    leftArm.setPower(leftPower);
                    rightArm.setPower(rightPower);
                })
                .waitSeconds(1)
                .strafeRight(oneTile)
                .build();



        /*
         * The INIT-loop:
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

        //The START command just came in: now work off the latest snapshot acquired during the init loop.

        /* Update the telemetry */
        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null){
            //default trajectory here if preferred
            drive.followTrajectorySequence(trajSeq2);

            telemetry.addData("Robot Position: ", "Not found :(");
            telemetry.update();
        } else if(tagOfInterest.id == LEFT){
            //left trajectory
            drive.followTrajectorySequence(trajSeq1);

            telemetry.addData("Robot Position: ", "Left");
            telemetry.update();
        } else if(tagOfInterest.id == MIDDLE){
            //middle trajectory
            drive.followTrajectorySequence(trajSeq2);

            telemetry.addData("Robot Position: ", "Middle");
            telemetry.update();
        } else{
            //right trajectory
            drive.followTrajectorySequence(trajSeq3);

            telemetry.addData("Robot Position: ", "Right");
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