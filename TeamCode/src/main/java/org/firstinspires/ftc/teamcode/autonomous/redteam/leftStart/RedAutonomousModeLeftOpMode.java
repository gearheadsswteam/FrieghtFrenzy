package org.firstinspires.ftc.teamcode.autonomous.redteam.leftStart;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.autonomous.redteam.RedTeamPositions;

@Autonomous(name = "RedAutonomousModeLeft", group = "Red")
public class RedAutonomousModeLeftOpMode extends AbstractAutonomousOpModeRR {
    int ringNum;

    private Pose2d initPos = RedTeamPositions.INIT_POSTION_LEFT;
    private String capstoneDetectorPosition;

    public RedAutonomousModeLeftOpMode() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPos);

        capstoneDetectorPosition = robot.capstoneDetector.getPosition();
        telemetry.addData("Capstone position ", capstoneDetectorPosition);
        telemetry.update();
        sleep(500);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void initOpModeAfterStart() {
        //robot.initCapstoneDetector();
    }

    @Override
    protected void executeOpMode() {
        while (this.opModeIsActive()) {
            capstoneDetectorPosition = robot.capstoneDetector.getPosition();
            telemetry.addData("Capstone position ", capstoneDetectorPosition);
            telemetry.update();
            sleep(1500);
        }

//        RedAutonomousLeft leftOpMode = new RedAutonomousLeft(mecanumDriveRR, autonomousRobotMover.robot, this);
//        leftOpMode.setLastPos(initPos);
//        leftOpMode.setCapStonePosition(capstoneDetectorPosition);
//        leftOpMode.executeOpMode();
    }
}
