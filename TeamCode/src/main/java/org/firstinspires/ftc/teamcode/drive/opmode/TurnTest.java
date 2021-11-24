package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        telemetry.addData("Heading: before", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.update();
        drive.turn(Math.toRadians(ANGLE));
        telemetry.addData("Heading: after", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.update();
    }
}
