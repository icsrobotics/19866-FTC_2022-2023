package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.autonomous.SandboxDetectPosition;

@Disabled
@Config
@TeleOp(name = "Red Terminal Autonomous", group = "Linear Opmode")
public class RedTerminalAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    SandboxDetectPosition detect = new SandboxDetectPosition(this);

    public int parkZone;

    public RedTerminalAuto (int parkZone) {
        this.parkZone = parkZone;
    }


    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        robot.init();
        detect.init();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Data", String.valueOf(detect.parkZone));
        }
    }
}
