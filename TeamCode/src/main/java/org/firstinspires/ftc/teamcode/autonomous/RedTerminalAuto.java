// Refrenced code: https://github.com/cobalt-colts/AprilTag-Workshop/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auton/AprilTagAutonomousInitDetectionExample.java
package org.firstinspires.ftc.teamcode.autonomous;

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
@Autonomous(name = "Red Terminal Autonomous", group = "Linear Opmode")
public class RedTerminalAuto extends LinearOpMode {
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

    //Drivetrain power values
    double oneTile = 1075;
    double errorRange = 15;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        robot.init();

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

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        double leftBlPower = DrivePIDController.PIDControl(-oneTile, robot.backLeft.getCurrentPosition(), errorRange);
        double leftFlPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), errorRange);
        double leftBrPower = DrivePIDController.PIDControl(-oneTile, robot.backRight.getCurrentPosition(), errorRange);
        double leftFrPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), errorRange);

        //Strafe right
        double rightBlPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), errorRange);
        double rightFlPower = DrivePIDController.PIDControl(-oneTile, robot.frontLeft.getCurrentPosition(), errorRange);
        double rightBrPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), errorRange);
        double rightFrPower = DrivePIDController.PIDControl(-oneTile, robot.frontRight.getCurrentPosition(), errorRange);

        //Forward
        double forwardBlPower = DrivePIDController.PIDControl(oneTile, robot.backLeft.getCurrentPosition(), errorRange);
        double forwardFlPower = DrivePIDController.PIDControl(oneTile, robot.frontLeft.getCurrentPosition(), errorRange);
        double forwardBrPower = DrivePIDController.PIDControl(oneTile, robot.backRight.getCurrentPosition(), errorRange);
        double forwardFrPower = DrivePIDController.PIDControl(oneTile, robot.frontRight.getCurrentPosition(), errorRange);

        //Backward
        double backBlPower = DrivePIDController.PIDControl(-oneTile, robot.backLeft.getCurrentPosition(), errorRange);
        double backFlPower = DrivePIDController.PIDControl(-oneTile, robot.frontLeft.getCurrentPosition(), errorRange);
        double backBrPower = DrivePIDController.PIDControl(-oneTile, robot.backRight.getCurrentPosition(), errorRange);
        double backFrPower = DrivePIDController.PIDControl(-oneTile, robot.frontRight.getCurrentPosition(), errorRange);

        if(tagOfInterest == null){
            //default trajectory here if preferred
            // FORWARD
            robot.frontRight.setPower(forwardBlPower);
            robot.frontLeft.setPower(forwardFlPower);
            robot.backRight.setPower(forwardBrPower);
            robot.backLeft.setPower(forwardFrPower);

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
        }else if(tagOfInterest.id != LEFT){
            //left trajectory
            // FORWARD
            robot.frontRight.setPower(forwardBlPower);
            robot.frontLeft.setPower(forwardFlPower);
            robot.backRight.setPower(forwardBrPower);
            robot.backLeft.setPower(forwardFrPower);

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
        }else if(tagOfInterest.id == MIDDLE){
            //middle trajectory
            // FORWARD
            robot.frontRight.setPower(forwardBlPower);
            robot.frontLeft.setPower(forwardFlPower);
            robot.backRight.setPower(forwardBrPower);
            robot.backLeft.setPower(forwardFrPower);

            //left tile
            robot.frontRight.setPower(leftBlPower);
            robot.frontLeft.setPower(leftFlPower);
            robot.backRight.setPower(leftBrPower);
            robot.backLeft.setPower(leftFrPower);

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
        }else{
            //right trajectory

            // FORWARD
            robot.frontRight.setPower(forwardBlPower);
            robot.frontLeft.setPower(forwardFlPower);
            robot.backRight.setPower(forwardBrPower);
            robot.backLeft.setPower(forwardFrPower);

            //two left tile
            robot.frontRight.setPower(leftBlPower);
            robot.frontLeft.setPower(leftFlPower);
            robot.backRight.setPower(leftBrPower);
            robot.backLeft.setPower(leftFrPower);
            sleep(1000);
            robot.frontRight.setPower(leftBlPower);
            robot.frontLeft.setPower(leftFlPower);
            robot.backRight.setPower(leftBrPower);
            robot.backLeft.setPower(leftFrPower);

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

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