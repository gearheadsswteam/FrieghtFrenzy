package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@Autonomous (name = "AutonomousBlue1")
public class AutonomousBlue1 extends LinearOpMode {
    Pose2d initPose = new Pose2d(-39, 62, -PI / 2);
    Pose2d[] grabPose = {new Pose2d(-33, 51, -1.15), new Pose2d(-33, 51, -PI / 2), new Pose2d(-41, 51, -PI / 2)};
    Pose2d dropPose = new Pose2d(-24, 33, -0.7);
    Pose2d spinnerPose = new Pose2d(-58, 57, -1.1);
    Pose2d parkPose = new Pose2d(-60, 33, PI);
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
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build(),
                drive.trajectoryBuilder(grabPose[1])
                .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[1]);})
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build(),
                drive.trajectoryBuilder(grabPose[2])
                .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[2]);})
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build()};
        Trajectory traj3 = drive.trajectoryBuilder(dropPose, true)
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(liftPositions[0]);
                    bucket.setPosition(bucketRest);
                    spinner.setPower(1);})
                .splineTo(spinnerPose.vec(), spinnerPose.getHeading() + PI)
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(spinnerPose)
                .addTemporalMarker(0, 0, () -> {
                    intake.setPower(INTAKE_SPEED);
                    gate.setPosition(gateDown);
                    spinner.setPower(0);})
                .setTangent(PI / 2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(new Pose2d(-53, 61, -PI / 2), 0)
                .splineToSplineHeading(new Pose2d(-45, 59, -2.1), 0)
                .resetVelConstraint()
                .setTangent(-PI / 2)
                .splineToSplineHeading(dropPose, dropPose.getHeading())
                .addTemporalMarker(1, -1, () -> {lift.setTargetPosition(liftPositions[2]);
                    bucket.setPosition(bucketUp);
                    intake.setPower(0);})
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(dropPose)
                .lineToLinearHeading(parkPose)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[0]);
                    bucket.setPosition(bucketRest);})
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
        drive.followTrajectory(traj5);
        ValueStorage.lastPose = drive.getPoseEstimate();
    }
}
