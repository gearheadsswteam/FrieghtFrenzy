package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

public class RedTeamPositions {
    //public static final Pose2d INIT_POSTION_LEFT = PoseStorage.DEFAULT_AUTONOMOUS_INIT_POSITION;
    public static final Pose2d ZERO_ZERO = new Pose2d(0, 0, 0);
    public static final Pose2d INIT_POSTION_LEFT = new Pose2d(-36, -60, Math.toRadians(90));
    public static final Pose2d RED_SHIPPING_HUB = new Pose2d(-12-8, -24+6, Math.toRadians(45));;
    public static final Pose2d PARK_ONLY_POSITION = new Pose2d(4.21-12, -36.82, 0);
    public static final Pose2d RED_CAROUSEL = new Pose2d(-60, -60, 0);


    public static final Pose2d INIT_POSTION_RIGHT = PoseStorage.DEFAULT_AUTONOMOUS_INIT_POSITION;

}
