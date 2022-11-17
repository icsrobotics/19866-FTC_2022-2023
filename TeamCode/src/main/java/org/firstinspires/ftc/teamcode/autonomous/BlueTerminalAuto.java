package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.autonomous.DetectPosition;

@Disabled
@Config
@TeleOp(name = "Blue Terminal Autonomous", group = "Linear Opmode")
public class BlueTerminalAuto extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    DetectPosition detection = new DetectPosition();

    public BlueTerminalAutoDriving


}
