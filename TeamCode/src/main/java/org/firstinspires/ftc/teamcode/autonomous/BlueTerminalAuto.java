package org.firstinspires.ftc.teamcode.autonomous;// Refrenced code: https://github.com/cobalt-colts/AprilTag-Workshop/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auton/AprilTagAutonomousInitDetectionExample.java

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MyPIDController;

import java.util.ArrayList;

@Config
@Autonomous(name = "Blue Terminal Autonomous", group = "Linear Opmode")
public class BlueTerminalAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    RobotHardware robot = new RobotHardware(this);


    MyPIDController DrivePIDController = new MyPIDController(0.07, 0.05, 0.01);
    MyPIDController ArmPIDController = new MyPIDController(0, 0, 0);

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

        //The INIT-loop: This REPLACES waitForStart!
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
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        int oneTile = 0;

        int lowTargetPosition = 0; //1936
        int midTargetPosition = 0; //2770
        int highTargetPosition = 0; //4075

        if(tagOfInterest == null || tagOfInterest.id == LEFT /* #1 */) {
            //intake cone
            robot.endServo.setPosition(1.0); //suck it in *schlorp*

            // strafe left one tile
            double blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            double brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            double flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            double frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            // forward one tile
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            //left 1/2 tile
            blPower = DrivePIDController.PIDControl(oneTile / 2, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile / 2, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile / 2, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile / 2, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            //raise arm to highest level
            double leftArmPower = ArmPIDController.PIDControl(highTargetPosition, robot.leftArmMotor.getCurrentPosition(), 50);
            double rightArmPower = ArmPIDController.PIDControl(highTargetPosition, robot.rightArmMotor.getCurrentPosition(), 50);

            robot.leftArmMotor.setPower(leftArmPower);
            robot.rightArmMotor.setPower(rightArmPower);

            sleep(500);

            // drop cone
            robot.endServo.setPosition(0); //spit it out *burp*

            sleep(500);

            // drop arm
            leftArmPower = ArmPIDController.PIDControl(0, robot.leftArmMotor.getCurrentPosition(), 50);
            rightArmPower = ArmPIDController.PIDControl(0, robot.rightArmMotor.getCurrentPosition(), 50);

            robot.leftArmMotor.setPower(leftArmPower);
            robot.rightArmMotor.setPower(rightArmPower);

            sleep(500);

            // strafe one tile right
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            // stay still
            blPower = 0;
            brPower = 0;
            flPower = 0;
            frPower = 0;

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            sleep(500);

            telemetry.addData("Robot", "LEFT OR NOT DETECTED");
            telemetry.addData("YOU!", "thank God if this code doesn't break the robot :)");
            telemetry.update();

        } else if(tagOfInterest.id == MIDDLE /* #2 */){
            //intake cone
            robot.endServo.setPosition(1.0); //suck it in *schlorp*

            // strafe left one tile
            double blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            double brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            double flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            double frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            // forward one tile
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            //left 1/2 tile
            blPower = DrivePIDController.PIDControl(oneTile / 2, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile / 2, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile / 2, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile / 2, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            //raise arm to highest level
            double leftArmPower = ArmPIDController.PIDControl(highTargetPosition, robot.leftArmMotor.getCurrentPosition(), 50);
            double rightArmPower = ArmPIDController.PIDControl(highTargetPosition, robot.rightArmMotor.getCurrentPosition(), 50);

            robot.leftArmMotor.setPower(leftArmPower);
            robot.rightArmMotor.setPower(rightArmPower);

            sleep(500);

            // drop cone
            robot.endServo.setPosition(0); //spit it out *burp*

            sleep(500);

            // drop arm
            leftArmPower = ArmPIDController.PIDControl(0, robot.leftArmMotor.getCurrentPosition(), 50);
            rightArmPower = ArmPIDController.PIDControl(0, robot.rightArmMotor.getCurrentPosition(), 50);

            robot.leftArmMotor.setPower(leftArmPower);
            robot.rightArmMotor.setPower(rightArmPower);

            sleep(500);

            // strafe one tile right
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            //strafe one tile right
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            telemetry.addData("Robot", "MIDDLE");
            telemetry.update();
        } else if (tagOfInterest.id == RIGHT /* #3 */){
            //intake cone
            robot.endServo.setPosition(1.0); //suck it in *schlorp*

            // strafe left one tile
            double blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            double brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            double flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            double frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            // forward one tile
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            //left 1/2 tile
            blPower = DrivePIDController.PIDControl(oneTile / 2, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile / 2, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile / 2, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile / 2, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            //raise arm to highest level
            double leftArmPower = ArmPIDController.PIDControl(highTargetPosition, robot.leftArmMotor.getCurrentPosition(), 50);
            double rightArmPower = ArmPIDController.PIDControl(highTargetPosition, robot.rightArmMotor.getCurrentPosition(), 50);

            robot.leftArmMotor.setPower(leftArmPower);
            robot.rightArmMotor.setPower(rightArmPower);

            sleep(500);

            // drop cone
            robot.endServo.setPosition(0); //spit it out *burp*

            sleep(500);

            // drop arm
            leftArmPower = ArmPIDController.PIDControl(0, robot.leftArmMotor.getCurrentPosition(), 50);
            rightArmPower = ArmPIDController.PIDControl(0, robot.rightArmMotor.getCurrentPosition(), 50);

            robot.leftArmMotor.setPower(leftArmPower);
            robot.rightArmMotor.setPower(rightArmPower);

            sleep(500);

            // strafe one tile right
            blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

            //strafe two tiles right
            blPower = DrivePIDController.PIDControl(oneTile * 2, robot.backLeft.getCurrentPosition(), 50);
            brPower = DrivePIDController.PIDControl(oneTile * 2, robot.backRight.getCurrentPosition(), 50);
            flPower = DrivePIDController.PIDControl(oneTile * 2, robot.frontLeft.getCurrentPosition(), 50);
            frPower = DrivePIDController.PIDControl(oneTile * 2, robot.frontRight.getCurrentPosition(), 50);

            robot.backLeft.setPower(blPower);
            robot.backRight.setPower(brPower);
            robot.frontLeft.setPower(flPower);
            robot.frontRight.setPower(frPower);

            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(500);

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
}
