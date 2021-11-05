package org.firstinspires.ftc.teamcode.autonomous.redteam.leftStart;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousHelper;
import org.firstinspires.ftc.teamcode.autonomous.redteam.RedTeamPositions;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.CapstoneDetector;
import org.firstinspires.ftc.teamcode.robot.actionparts.DeliveryArmSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.DuckrotationSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;

class RedAutonomousLeft {

    private MecanumDriveRR mecanumDriveRR;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;
    private String capStonePosition;
    DeliveryArmSystem deliveryArmSystem;
    DuckrotationSystem duckrotationSystem;
    Intakesystem intakesystem;

    public RedAutonomousLeft(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.mecanumDriveRR = mecanumDriveRR;
        this.currOpMode = currOpMode;
        this.initPos = RedTeamPositions.INIT_POSTION_LEFT;
        deliveryArmSystem = gearheadsMecanumRobotRR.deliveryArmSystem;
        duckrotationSystem = gearheadsMecanumRobotRR.duckrotationSystem;
        intakesystem = gearheadsMecanumRobotRR.intakesystem;

    }

    public void setLastPos(Pose2d lastKnownPos) {
        this.lastPos = lastKnownPos;
    }

    public void setCapStonePosition(String capStonePosition){
        this.capStonePosition = capStonePosition;
    }

    public void executeOpMode() {

        //From Starting position to Case 0 drop zone
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, Math.toRadians(90))
                .splineTo(AutonomousHelper.getVector2d(RedTeamPositions.RED_SHIPPING_HUB),Math.toRadians(90))
                .build();

        mecanumDriveRR.followTrajectory(traj1);

        currOpMode.sleep(3000);

        Trajectory traj2= mecanumDriveRR.trajectoryBuilder(traj1.end(), true)
                .splineTo(AutonomousHelper.getVector2d(RedTeamPositions.RED_CAROUSEL),RedTeamPositions.RED_CAROUSEL.getHeading())
                .build();

        mecanumDriveRR.followTrajectory(traj2);
    }

    protected void startIntake(){
        intakesystem.startInTake();
    }

    protected void stopIntake(){
        intakesystem.stopInTake();
    }

    protected void deliverCargoToRightLevel(){
        if(CapstoneDetector.POSITION_A.equals(capStonePosition)){
            deliveryArmSystem.setLiftElevatorLow();
        }else if(CapstoneDetector.POSITION_A.equals(capStonePosition)){
            deliveryArmSystem.setLiftElevatorLow();
        }else if(CapstoneDetector.POSITION_A.equals(capStonePosition)){
            deliveryArmSystem.setLiftElevatorLow();
        }
    }

    protected void rotateCarousel(){
        duckrotationSystem.rotateClockWiseOneRotation();
    }
}
