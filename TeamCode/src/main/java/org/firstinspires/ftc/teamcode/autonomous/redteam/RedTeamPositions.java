package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

public class RedTeamPositions {
    public static final Pose2d INIT_POS = PoseStorage.DEFAULT_AUTONOMOUS_INIT_POSITION;
    public static final Pose2d WG2_START_POS = new Pose2d(-60, -24, 0);;
    public static final Pose2d PARK_ONLY_POSITION = new Pose2d(4.21-12, -36.82, 0);
    public static final Pose2d SHOOTING_POS = new Pose2d(4.21-12, -36.82, 0);
}
