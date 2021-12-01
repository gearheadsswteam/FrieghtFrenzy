package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
public class ValueStorage {
    static final double armRest = 0.43;
    static final double armDown = 0.795;
    static final double armUp = 0.65;
    static final double clawOpen = 0.43;
    static final double clawClosed = 0.76;
    static final double bucketRest = 0.36;
    static final double bucketUp = 0.58;
    static final double bucketDown = 0.80;
    static final double gateDown = 0.31;
    static final double gateUp = 0.65;
    static final double bucketSensorThreshold = 4.5;
    static final double[] armStateTimes = {1000, 1000, 1000, 1000, 1000, 1000};
    static final double[][] intakeStateTimes = {{500, 750, 1000}, {250, 250, 250}, {250, 250, 500}, {250, 250, 250}};
    static final int bucketDetectionFrames = 10;
    static final int cameraDetectionFrames = 10;
    static final int[] liftPositions = {0, -360, -940};
    static int lastIntakeState = 0;
    static int lastLiftState = 0;
    static int lastArmState = 0;
    public static int redMultiplier = 1;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}
