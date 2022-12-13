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
@Autonomous(name = "Blue Terminal Autonomous", group = "Linear Opmode")
public class BlueTerminalAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    RobotHardware robot = new RobotHardware(this);

    MyPIDController DrivePIDController = new MyPIDController(0.07, 0.05, 0.01);
    MyPIDController ArmPIDController = new MyPIDController(0.05, 0, 0);


    // PUT CAMERA IN USB 2.0 PORT!!!
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    static final double FEET_PER_METER = 3.28084;

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
                        break;
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest");
            }

            telemetry.update();
            sleep(20);
        }

        //The START command just came in: now work off the latest snapshot acquired during the init loop.

        //ACTUAL CODE
        //GENERAL:
            // suck in cone
            // left tile
            //forward tile
            // left 1/2 tile
            //raise arm
            //drop cone
            // drop arm
            //right 1/2 tile

        if(tagOfInterest == null || tagOfInterest.id == LEFT /*#1*/) {
            // stay still

            telemetry.addData("Robot", "LEFT OR NOT DETECTED");
            telemetry.update();

        } else if(tagOfInterest.id == MIDDLE /* #2 */){
            //right tile

            telemetry.addData("Robot", "MIDDLE");
            telemetry.update();
        } else if (tagOfInterest.id == RIGHT /*#3*/){
            //two right tile

            telemetry.addData("Robot", "RIGHT");
            telemetry.update();
        }
    }
}
