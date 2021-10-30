package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.CapstoneDetector;

@Autonomous(name = "RedAutonomousModeRR", group = "Red")
public class RedAutonomousModeRR extends AbstractAutonomousOpModeRR {
    int ringNum;

    Pose2d initPos = new Pose2d(-60, -48, 0);
    CapstoneDetector capstoneDetector;

    public RedAutonomousModeRR() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPos);
        capstoneDetector = this.robot.capstoneDetector;
        //capstoneDetector.intitalize(this);
        String capstonePosition = capstoneDetector.getPosition();

        telemetry.addData("Capstone postion ", capstonePosition);
        telemetry.update();
        sleep(500);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {

        String capstonePosition = capstoneDetector.getPosition();
        telemetry.addData("Capstone postion ", capstonePosition);
        telemetry.update();
        sleep(500);


        if (CapstoneDetector.POSITION_A.equals(capstonePosition)) {


        } else if (CapstoneDetector.POSITION_B.equals(capstonePosition)) {

        }
        if (CapstoneDetector.POSITION_C.equals(capstonePosition)) {

        }
    }
}
