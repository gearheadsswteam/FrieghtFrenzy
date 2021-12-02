package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
public class ValueStorage {
    public static final double armRest = 0.43;
    public static final double armDown = 0.795;
    public static final double armUp = 0.65;
    public static final double clawOpen = 0.43;
    public static final double clawClosed = 0.76;
    public static final double bucketRest = 0.36;
    public static final double bucketUp = 0.58;
    public static final double bucketDown = 0.80;
    public static final double gateDown = 0.31;
    public static final double gateUp = 0.65;
    public static final double bucketSensorThreshold = 4.5;
    public static final double[] armStateTimes = {1000, 1000, 1000, 1000, 1000, 1000};
    public static final double[][] intakeStateTimes = {{500, 750, 1000}, {250, 250, 250}, {250, 250, 500}, {250, 250, 250}};
    public static final int bucketDetectionFrames = 10;
    public static final int cameraDetectionFrames = 10;
    public static final int[] liftPositions = {0, -360, -940};
    public static int lastIntakeState = 0;
    public static int lastLiftState = 0;
    public static int lastArmState = 0;
    public static int redMultiplier = 1;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}
