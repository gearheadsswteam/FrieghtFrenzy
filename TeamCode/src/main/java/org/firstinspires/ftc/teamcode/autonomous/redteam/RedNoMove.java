package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpModeRR;

@Autonomous(name = "RedNoMove", group = "Red")
public class RedNoMove extends AbstractAutonomousOpModeRR {
    private Pose2d initPos = RedTeamPositions.WG2_START_POS;
    private Pose2d destPos = RedTeamPositions.SHOOTING_POS;
    private long delayMs = 5000;

    public RedNoMove() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        //No op
    }
}
