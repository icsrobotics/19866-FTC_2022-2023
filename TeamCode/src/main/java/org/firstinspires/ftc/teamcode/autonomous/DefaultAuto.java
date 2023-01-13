package org.firstinspires.ftc.teamcode.autonomous;

// Refrenced code: https://github.com/cobalt-colts/AprilTag-Workshop/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auton/AprilTagAutonomousInitDetectionExample.java


import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RESET_ENCODERS;

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
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.MyPIDController;

import java.util.ArrayList;

@Config
@Autonomous(name = "Defualt Autonomous", group = "Linear Opmode")
public class DefaultAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    RobotHardware robot = new RobotHardware(this);
    MyPIDController DrivePIDController = new MyPIDController(0.07, 0.05, 0.01);

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

    // Tag ID 1,2,3 from the 36h11 family. Ours is 6, 7, 8
    int LEFT = 6;
    int MIDDLE = 7;
    int RIGHT = 8;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

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
        robot.init();

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

        /* Actually do something useful */
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
        double oneTile = 1075;
        double errorRange = 10;

        //Power
        double blPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), errorRange);
        double flPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), errorRange);
        double brPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), errorRange);
        double frPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), errorRange);

        if(tagOfInterest == null){
            //default trajectory here if preferred
            // FORWARD
            robot.frontRight.setPower(frPower);
            robot.frontLeft.setPower(flPower);
            robot.backRight.setPower(brPower);
            robot.backLeft.setPower(blPower);

        }else if(tagOfInterest.id == LEFT /**/){
            //left trajectory - 1

            // FORWARD
            robot.frontRight.setPower(frPower);
            robot.frontLeft.setPower(flPower);
            robot.backRight.setPower(brPower);
            robot.backLeft.setPower(blPower);

            //LEFT
            robot.frontRight.setPower(frPower);
            robot.frontLeft.setPower(-flPower);
            robot.backRight.setPower(-brPower);
            robot.backLeft.setPower(blPower);

        }else if(tagOfInterest.id == MIDDLE){
            //middle trajectory - 2

            // FORWARD
            robot.frontRight.setPower(frPower);
            robot.frontLeft.setPower(flPower);
            robot.backRight.setPower(brPower);
            robot.backLeft.setPower(blPower);

        }else{
            //right trajectory - 3

            // FORWARD
            robot.frontRight.setPower(frPower);
            robot.frontLeft.setPower(flPower);
            robot.backRight.setPower(brPower);
            robot.backLeft.setPower(blPower);

            //RIGHT
            robot.frontRight.setPower(-frPower);
            robot.frontLeft.setPower(flPower);
            robot.backRight.setPower(brPower);
            robot.backLeft.setPower(-blPower);

            telemetry.addData("Robot", "RIGHT");
            telemetry.update();
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
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
    /*public void strafeLeft(){
        robot.frontRight.setPower(leftBlPower);
        robot.frontLeft.setPower(leftFlPower);
        robot.backRight.setPower(leftBrPower);
        robot.backLeft.setPower(leftFrPower);
    }

    public void strafeRight(){
        robot.frontRight.setPower(rightBlPower);
        robot.frontLeft.setPower(rightFlPower);
        robot.backRight.setPower(rightBrPower);
        robot.backLeft.setPower(rightFrPower);
    }

    public void moveForward(){
        robot.frontRight.setPower(forwardBlPower);
        robot.frontLeft.setPower(forwardFlPower);
        robot.backRight.setPower(forwardBrPower);
        robot.backLeft.setPower(forwardFrPower);
    }

    public void moveBackward(){
        robot.frontRight.setPower(backBlPower);
        robot.frontLeft.setPower(backFlPower);
        robot.backRight.setPower(backBrPower);
        robot.backLeft.setPower(backFrPower);
    }

    public void powerZero(){
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }*/

