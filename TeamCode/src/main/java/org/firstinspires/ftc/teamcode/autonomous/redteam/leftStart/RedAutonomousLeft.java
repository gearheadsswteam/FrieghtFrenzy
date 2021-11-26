package org.firstinspires.ftc.teamcode.autonomous.redteam.leftStart;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousHelper;
import org.firstinspires.ftc.teamcode.autonomous.redteam.RedTeamPositions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.CapstoneDetector;
import org.firstinspires.ftc.teamcode.robot.actionparts.DeliveryArmSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.DuckrotationSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static java.lang.Math.PI;

class RedAutonomousLeft {

    private MecanumDriveRR drive;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;
    private String capStonePosition;
    DeliveryArmSystem deliveryArmSystem;
    DuckrotationSystem duckrotationSystem;
    Intakesystem intakesystem;

    public RedAutonomousLeft(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.drive = mecanumDriveRR;
        this.currOpMode = currOpMode;
        this.initPos = RedTeamPositions.INIT_POSTION_LEFT;
        deliveryArmSystem = gearheadsMecanumRobotRR.deliveryArmSystem;
        duckrotationSystem = gearheadsMecanumRobotRR.duckrotationSystem;
        intakesystem = gearheadsMecanumRobotRR.intakesystem;

    }

    public void setLastPos(Pose2d lastKnownPos) {
        this.lastPos = lastKnownPos;
    }

    public void setCapStonePosition(String capStonePosition) {
        this.capStonePosition = capStonePosition;
    }

    public void executeOpMode() {
        Trajectory[] traj1 = {drive.trajectoryBuilder(RedTeamPositions.initPose)
                .lineToLinearHeading(RedTeamPositions.grabPose[0])
                .build(),
                drive.trajectoryBuilder(RedTeamPositions.initPose)
                        .lineToLinearHeading(RedTeamPositions.grabPose[1])
                        .build(),
                drive.trajectoryBuilder(RedTeamPositions.initPose)
                        .lineToLinearHeading(RedTeamPositions.grabPose[2])
                        .build()};

        Trajectory[] traj2 = {drive.trajectoryBuilder(RedTeamPositions.grabPose[0])
                .splineTo(RedTeamPositions.dropPose.vec(), RedTeamPositions.dropPose.getHeading())
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build(),
                drive.trajectoryBuilder(RedTeamPositions.grabPose[1])
                        .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[1]);})
                        .splineTo(RedTeamPositions.dropPose.vec(), RedTeamPositions.dropPose.getHeading())
                        .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                        .build(),
                drive.trajectoryBuilder(RedTeamPositions.grabPose[2])
                        .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[2]);})
                        .splineTo(RedTeamPositions.dropPose.vec(), RedTeamPositions.dropPose.getHeading())
                        .addTemporalMarker(1, -0.25, () -> {bucket.setPosition(bucketDown);})
                        .build()};
        Trajectory traj3 = drive.trajectoryBuilder(RedTeamPositions.dropPose, true)
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(liftPositions[0]);
                    bucket.setPosition(bucketRest);
                    spinner.setPower(1);})
                .splineTo(RedTeamPositions.spinnerPose.vec(), RedTeamPositions.spinnerPose.getHeading() + PI)
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(RedTeamPositions.spinnerPose)
                .addTemporalMarker(0, 0, () -> {
                    intake.setPower(0.5);
                    spinner.setPower(0);})
                .setTangent(-PI / 2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(new Pose2d(-53, -61, PI / 2), 0)
                .splineToSplineHeading(new Pose2d(-40, -59, 2.2), 0)
                .resetVelConstraint()
                .splineToSplineHeading(dropPose2, 0.9)
                .addTemporalMarker(1, -1, () -> {lift.setTargetPosition(liftPositions[2]);
                    bucket.setPosition(bucketUp);
                    intake.setPower(0);})
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(dropPose2)
                .lineToLinearHeading(parkPose)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[0]);
                    bucket.setPosition(bucketRest);})
                .build();
    }

    protected void startIntake() {
        intakesystem.startInTake();
    }

    protected void stopIntake() {
        intakesystem.stopInTake();
    }

    protected void setElevatorToBarcodeLevel() {
        if (CapstoneDetector.POSITION_A.equals(capStonePosition)) {
            deliveryArmSystem.setLiftElevatorLow();
        } else if (CapstoneDetector.POSITION_A.equals(capStonePosition)) {
            deliveryArmSystem.setLiftElevatorLow();
        } else if (CapstoneDetector.POSITION_A.equals(capStonePosition)) {
            deliveryArmSystem.setLiftElevatorLow();
        }
    }

    protected void resetElevator() {
        deliveryArmSystem.setLiftElevatorLow();
    }

    protected void deliverCargo() {
        deliveryArmSystem.tiltBucket();
        currOpMode.sleep(500);
        deliveryArmSystem.unTiltBucket();
    }

    protected void rotateCarousel() {
        duckrotationSystem.rotateClockWise();
    }
}
