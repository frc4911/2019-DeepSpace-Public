package com.team4911.frc2019.paths;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team4911.frc2019.Constants;

public class FieldPoses {
    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.

    // These are actual field locations per FIRST issued drawings.

    public static final Pose2d kFeederRight = new Pose2d(new Translation2d(0, -136.2), Rotation2d.fromDegrees(180));

    public static final Pose2d kRocketRightNear = new Pose2d(new Translation2d(213.57, -144.29), Rotation2d.fromDegrees(-30));
    public static final Pose2d kRocketRightCenter = new Pose2d(new Translation2d(229.19, -134.59), Rotation2d.fromDegrees(-90));
    public static final Pose2d kRocketRightFar = new Pose2d(new Translation2d(244.69, -144.29), Rotation2d.fromDegrees(-150));

    public static final Pose2d kCargoRightFar = new Pose2d(new Translation2d(304.25, -28.87), Rotation2d.fromDegrees(-90));
    public static final Pose2d kCargoRightCenter = new Pose2d(new Translation2d(282.5, -28.87), Rotation2d.fromDegrees(-90));
    public static final Pose2d kCargoRightNear = new Pose2d(new Translation2d(260.95, -28.87), Rotation2d.fromDegrees(-90));
    public static final Pose2d kCargoFrontRight = new Pose2d(new Translation2d(220.25, -10.9), Rotation2d.fromDegrees(0));

    public static final Pose2d kCargoFrontLeft = new Pose2d(new Translation2d(220.25, 10.9), Rotation2d.fromDegrees(0));
    public static final Pose2d kCargoLeftNear = new Pose2d(new Translation2d(260.95, 28.87), Rotation2d.fromDegrees(90));
    public static final Pose2d kCargoLeftCenter = new Pose2d(new Translation2d(282.5, 28.87), Rotation2d.fromDegrees(90));
    public static final Pose2d kCargoLeftFar = new Pose2d(new Translation2d(304.25, 28.87), Rotation2d.fromDegrees(90));

    public static final Pose2d kRocketLeftNear = new Pose2d(new Translation2d(213.57, 144.29), Rotation2d.fromDegrees(30));
    public static final Pose2d kRocketLeftCenter = new Pose2d(new Translation2d(229.19, 134.59), Rotation2d.fromDegrees(90));
    public static final Pose2d kRocketLeftFar = new Pose2d(new Translation2d(244.69, 144.29), Rotation2d.fromDegrees(150));

    public static final Pose2d kFeederLeft = new Pose2d(new Translation2d(0, 136.2), Rotation2d.fromDegrees(180));


    // This pose is used to calculate a safe position close to the target with the same pose as target.
    public static final Pose2d kSafeDistance = new Pose2d(new Translation2d(Constants.kCenterToFrontBumperDistance + 3, 0), new Rotation2d());


    public static Pose2d getSafePose(Pose2d origin) {
        return origin.transformBy(kSafeDistance);
    }
}


