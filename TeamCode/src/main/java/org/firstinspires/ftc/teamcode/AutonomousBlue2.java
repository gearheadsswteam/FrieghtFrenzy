package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.*;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous (name = "AutonomousBlue2")
public class AutonomousBlue2 extends LinearOpMode {
    Pose2d initPose = new Pose2d(-39, 62, -PI / 2);
    Pose2d[] grabPose = {new Pose2d(-33, 51, -1.15), new Pose2d(-33, 51, -PI / 2), new Pose2d(-41, 51, -PI / 2)};
    Pose2d dropPose1 = new Pose2d(-24, 33, -0.7);
    Pose2d spinnerPose = new Pose2d(-58, 57, -1.1);
    Pose2d intakePose1 = new Pose2d(16, 34, 0);
    Pose2d dropPose2 = new Pose2d(-1, 38, -2);
    Pose2d intakePose2 = new Pose2d(48, 65, PI);
    Pose2d parkPose = new Pose2d(38, 65, PI);
    SampleMecanumDrive drive;
    ShippingElementDetector camera;
    DcMotorEx intake;
    DcMotorEx lift;
    Servo arm;
    Servo claw;
    Servo bucket;
    Servo gate;
    CRServo spinner;
    String caseDetected = "C";
    String caseSet = "C";
    int detectionFrames = 0;
    double INTAKE_SPEED = 1;
    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new ShippingElementDetector(hardwareMap, -1);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        gate = hardwareMap.get(Servo.class, "gate");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        intake.setDirection(REVERSE);
        spinner.setDirection(REVERSE);
        drive.setPoseEstimate(initPose);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        camera.initialize();
        Trajectory[] traj1 = {drive.trajectoryBuilder(initPose)
                .lineToLinearHeading(grabPose[0])
                .build(),
                drive.trajectoryBuilder(initPose)
                .lineToLinearHeading(grabPose[1])
                .build(),
                drive.trajectoryBuilder(initPose)
                .lineToLinearHeading(grabPose[2])
                .build()};
        Trajectory[] traj2 = {drive.trajectoryBuilder(grabPose[0])
                .splineTo(dropPose1.vec(), dropPose1.getHeading())
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build(),
                drive.trajectoryBuilder(grabPose[1])
                .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[1]);})
                .splineTo(dropPose1.vec(), dropPose1.getHeading())
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build(),
                drive.trajectoryBuilder(grabPose[2])
                .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[2]);})
                .splineTo(dropPose1.vec(), dropPose1.getHeading())
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build()};
        Trajectory traj3 = drive.trajectoryBuilder(dropPose1, true)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[0]);
                    bucket.setPosition(bucketRest);
                    spinner.setPower(1);})
                .splineTo(spinnerPose.vec(), spinnerPose.getHeading() + PI)
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(spinnerPose)
                .addTemporalMarker(0, 0, () -> {
                    intake.setPower(INTAKE_SPEED*0.6);///This is to grab the alliance duck
                    gate.setPosition(gateDown);
                    spinner.setPower(0);})
                .setTangent(PI / 2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(new Pose2d(-53, 61, -PI / 2), 0)
                .splineToSplineHeading(new Pose2d(-45, 59, -2.1), 0)
                .resetVelConstraint()
                .setTangent(-PI / 2)
                .splineToSplineHeading(dropPose1, dropPose1.getHeading())
                .addTemporalMarker(1, -1, () -> {lift.setTargetPosition(liftPositions[2]);
                    bucket.setPosition(bucketUp);
                    intake.setPower(0);})
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build();
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(dropPose1)
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[0]);
                    bucket.setPosition(bucketRest);
                    intake.setPower(INTAKE_SPEED);})
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(-11, 40), 0)
                .resetVelConstraint()
                .splineTo(intakePose1.vec(), intakePose1.getHeading())
                .setReversed(false)
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .addTemporalMarker(1, -1, () -> {lift.setTargetPosition(liftPositions[2]);
                    bucket.setPosition(bucketUp);})
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build();
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(dropPose2)
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[0]);
                    bucket.setPosition(bucketRest);
                    intake.setPower(INTAKE_SPEED);
                    gate.setPosition(gateUp);})
                .splineToSplineHeading(new Pose2d(9, 57, PI), PI - 2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(13, 65), 0)
                .lineTo(intakePose2.vec())
                .build();
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(intakePose2)
                .addTemporalMarker(0.5, () -> {gate.setPosition(gateDown);})
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(13, 66), PI)
                .splineToConstantHeading(new Vector2d(9, 57), 2)
                .resetVelConstraint()
                .splineToSplineHeading(dropPose2, dropPose2.getHeading())
                .addTemporalMarker(1, -1, () -> {lift.setTargetPosition(liftPositions[2]);
                    bucket.setPosition(bucketUp);
                    intake.setPower(0);})
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(dropPose2)
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[0]);
                    bucket.setPosition(bucketRest);})
                .splineToSplineHeading(new Pose2d(9, 57, PI), PI - 2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(13, 65), 0)
                .lineTo(parkPose.vec())
                .build();
        while (!isStarted() && !isStopRequested()) {
            if (camera.caseDetected() == caseDetected) {
                detectionFrames++;
            } else {
                caseDetected = camera.caseDetected();
                detectionFrames = 1;
            }
            if (detectionFrames >= cameraDetectionFrames) {
                caseSet = caseDetected;
            }
            telemetry.addData("Case:", caseSet);
            telemetry.update();
        }
        ValueStorage.lastIntakeState = 0;
        ValueStorage.lastLiftState = 0;
        ValueStorage.lastArmState = 0;
        ValueStorage.redMultiplier = -1;
        ValueStorage.lastPose = parkPose;
        camera.end();
        lift.setPower(1);
        if (caseSet == "A") {
            arm.setPosition(armDown);
            claw.setPosition(clawOpen);
            sleep(1800);
            drive.followTrajectory(traj1[0]);

            claw.setPosition(clawClosed);
            sleep(500);
            arm.setPosition(armRest);
            sleep(500);
            drive.followTrajectory(traj2[0]);
        } else if (caseSet == "B") {
            arm.setPosition(armDown);
            claw.setPosition(clawOpen);
            sleep(1800);
            drive.followTrajectory(traj1[1]);

            claw.setPosition(clawClosed);
            sleep(500);
            arm.setPosition(armRest);
            sleep(500);
            drive.followTrajectory(traj2[1]);
        } else {
            arm.setPosition(armDown);
            claw.setPosition(clawOpen);
            sleep(1800);
            drive.followTrajectory(traj1[2]);

            claw.setPosition(clawClosed);
            sleep(500);
            arm.setPosition(armRest);
            sleep(500);
            drive.followTrajectory(traj2[2]);
        }
        sleep(500);
        drive.followTrajectory(traj3);
        sleep(3000);
        drive.followTrajectorySequence(traj4);
        sleep(500);
        drive.followTrajectorySequence(traj5);
        sleep(500);
        drive.followTrajectorySequence(traj6);
        drive.followTrajectorySequence(traj7);
        sleep(500);
        drive.followTrajectorySequence(traj8);
        ValueStorage.lastPose = drive.getPoseEstimate();
    }
}
