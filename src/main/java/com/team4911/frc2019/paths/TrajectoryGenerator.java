package com.team4911.frc2019.paths;

import com.team4911.frc2019.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 96.0;
    private static final double kMaxAccel = 96.0;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0;
    private static final double kMaxCentripetalAccel = 100.0;
    private static final double kMaxVoltage = 9.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 130.0;
    private static final double kFirstPathMaxVel = 130.0;

    private static final double kSimpleSwitchMaxAccel = 100.0;
    private static final double kSimpleSwitchMaxCentripetalAccel = 80.0;
    private static final double kSimpleSwitchMaxVelocity = 120.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)

    // Cargo Ship Paths
    public static final Pose2d kShipBayFarPose = new Pose2d(new Translation2d(302.0, -45.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kSpacePose = new Pose2d(new Translation2d(175.0, -110.0), Rotation2d.fromDegrees(10.0));
    public static final Pose2d kLoadStationPose = new Pose2d(new Translation2d(20.0, -135.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kSpacePose2 = new Pose2d(new Translation2d(175.0, -110.0), Rotation2d.fromDegrees(190.0));
    public static final Pose2d kShipBayMidPose = new Pose2d(new Translation2d(280.0, -45.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kSpacePose3 = new Pose2d(new Translation2d(280.0, -57.0), Rotation2d.fromDegrees(90.0));

    public static final Pose2d kLvl1Pose = new Pose2d(new Translation2d(65.25, -48.75), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kShipBayNearPose = new Pose2d(new Translation2d(265.0, -41.0), Rotation2d.fromDegrees(90.0)); // tune this!
    public static final Pose2d kStationPose = new Pose2d(new Translation2d(289.0, -80.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kStationPose2 = new Pose2d(new Translation2d(30.0, -129.0), Rotation2d.fromDegrees(180.0)); // tune this!
    // public static final Pose2d kShipBayMid = new Pose2d(new Translation2d(175.0, -100.0), Rotation2d.fromDegrees(210.0));
    public static final Pose2d kShipBayMid2 = new Pose2d(new Translation2d(290.0, -110.0), Rotation2d.fromDegrees(135.0));
    // public static final Pose2d kShipBayMid2 = new Pose2d(new Translation2d(285.0, -90.0), Rotation2d.fromDegrees(90.0));
    public static final Pose2d kShipBayMid3 = new Pose2d(new Translation2d(300.0, -41.0), Rotation2d.fromDegrees(90.0)); // tune this!

    // Rocket Near Paths
    public static final Pose2d kLeftLvl1Pose = new Pose2d(new Translation2d(68.0, 45.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kLeftRocketNearPose = new Pose2d(new Translation2d(200.0, 135.0), Rotation2d.fromDegrees(30.0));
    public static final Pose2d kLeftRocketNearPose2 = new Pose2d(new Translation2d(200.0 + 10.0, 135.0 + 16.0), Rotation2d.fromDegrees(30.0));
    public static final Pose2d kLoadStationPose2 = new Pose2d(new Translation2d(115.0, 118.0), Rotation2d.fromDegrees(350.0));
    public static final Pose2d kLoadStationPose4 = new Pose2d(new Translation2d(115.0, 118.0), Rotation2d.fromDegrees(170.0));
    public static final Pose2d kLoadStationPose3 = new Pose2d(new Translation2d(30.0, 135.0), Rotation2d.fromDegrees(180.0));

    public static final Pose2d kRightLvl1Pose = new Pose2d(new Translation2d(68.0, -45.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kRightRocketNearPose = new Pose2d(new Translation2d(200.0, -140.0), Rotation2d.fromDegrees(330.0));
    public static final Pose2d kRightRocketNearPose2 = new Pose2d(new Translation2d(200.0 + 7.0, -140.0 + 2.0), Rotation2d.fromDegrees(330.0));
    public static final Pose2d kRightLoadStationPose2 = new Pose2d(new Translation2d(115.0, -118.0), Rotation2d.fromDegrees(10.0));
    public static final Pose2d kRightLoadStationPose4 = new Pose2d(new Translation2d(115.0, -118.0), Rotation2d.fromDegrees(190.0));
    public static final Pose2d kRightLoadStationPose3 = new Pose2d(new Translation2d(30.0, -135.0), Rotation2d.fromDegrees(180.0));

    // Rocket Far Paths

    // Test Paths
    public static final Pose2d kTestPose8 = new Pose2d(new Translation2d(143.0, 103.0), Rotation2d.fromDegrees(30.0));
    public static final Pose2d kTestPose1 = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPose2 = new Pose2d(new Translation2d(60.0, 0.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPose3 = new Pose2d(new Translation2d(120.0, 0.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kTestPose4 = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kTestPose5 = new Pose2d(new Translation2d(68.0, 45.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d kTestPose6 = new Pose2d(new Translation2d(260.0, 45.0), Rotation2d.fromDegrees(270.0));
    public static final Pose2d kTestPose7 = new Pose2d(new Translation2d(260.0, 81.0), Rotation2d.fromDegrees(270.0));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        // Cargo Ship Trajectories
        public final MirroredTrajectory lvl1ToShipBayFar;
        public final MirroredTrajectory shipBayFarToLoadStation;
        public final MirroredTrajectory shipBayFarToLoadStation2;
        public final MirroredTrajectory loadStationToShipBayMid;
        public final MirroredTrajectory loadStationToShipBayMid2;
        public final MirroredTrajectory backUpFromShip;

        public final MirroredTrajectory lvl1ToShipBayNear;
        public final MirroredTrajectory shipBayNearToStation;
        public final MirroredTrajectory shipBayNearToStation2;
        public final MirroredTrajectory stationToShipBayMid;
        public final MirroredTrajectory stationToShipBayMid2;
        public final MirroredTrajectory shipBayMidToStation;

        // Rocket Near Trajectories
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftLvl1ToLeftRocket;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftRocketToLoadStation;
        public final Trajectory<TimedState<Pose2dWithCurvature>> leftRocketToLoadStation2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> loadStationToLeftRocket;
        public final Trajectory<TimedState<Pose2dWithCurvature>> loadStationToLeftRocket2;

        public final MirroredTrajectory rightLvl1ToRightRocket;
        public final MirroredTrajectory rightRocketToLoadStation;
        public final MirroredTrajectory rightRocketToLoadStation2;
        public final MirroredTrajectory rightLoadStationToRightRocket;
        public final MirroredTrajectory rightLoadStationToRightRocket2;

        // Rocket Far Trajectories

        // Test Trajectories
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectory1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectory2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectory3;

        private TrajectorySet() {
            lvl1ToShipBayFar = new MirroredTrajectory(getLvl1ToShipBayFar());
            shipBayFarToLoadStation = new MirroredTrajectory(getShipBayFarToLoadStation());
            shipBayFarToLoadStation2 = new MirroredTrajectory(getShipBayFarToLoadStation2());
            loadStationToShipBayMid = new MirroredTrajectory(getLoadStationToShipBayMid());
            loadStationToShipBayMid2 = new MirroredTrajectory(getLoadStationToShipBayMid2());
            backUpFromShip = new MirroredTrajectory(getBackUpFromShip());

            leftLvl1ToLeftRocket = getLeftLvl1ToLeftRocket();
            leftRocketToLoadStation = getLeftRocketToLoadStation();
            leftRocketToLoadStation2 = getLeftRocketToLoadStation2();
            loadStationToLeftRocket = getLoadStationToLeftRocket();
            loadStationToLeftRocket2 = getLoadStationToLeftRocket2();

            rightLvl1ToRightRocket = new MirroredTrajectory(getLvl1ToRocket());
            rightRocketToLoadStation = new MirroredTrajectory(getRocketToLoadStation());
            rightRocketToLoadStation2 = new MirroredTrajectory(getRocketToLoadStation2());
            rightLoadStationToRightRocket = new MirroredTrajectory(getLoadStationToRocket());
            rightLoadStationToRightRocket2 = new MirroredTrajectory(getLoadStationToRocket2());

            lvl1ToShipBayNear = new MirroredTrajectory(getLvl1ToShipBayNear());
            shipBayNearToStation = new MirroredTrajectory(getShipBayNearToStation());
            shipBayNearToStation2 = new MirroredTrajectory(getShipBayNearToStation2());
            stationToShipBayMid = new MirroredTrajectory(getStationToShipBayMid());
            stationToShipBayMid2 = new MirroredTrajectory(getStationToShipBayMid2());
            shipBayMidToStation = new MirroredTrajectory(getShipBayMidToStation());

            testTrajectory1 = getTestPath1();
            testTrajectory2 = getTestPath2();
            testTrajectory3 = getTestPath3();
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl1ToShipBayFar() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftLvl1Pose);
            waypoints.add(kShipBayFarPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getShipBayFarToLoadStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kShipBayFarPose);
            waypoints.add(kSpacePose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getShipBayFarToLoadStation2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSpacePose);
            waypoints.add(kLoadStationPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadStationToShipBayMid() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadStationPose);
            waypoints.add(kSpacePose2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadStationToShipBayMid2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSpacePose2);
            waypoints.add(kShipBayMidPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackUpFromShip() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kShipBayMidPose);
            waypoints.add(kSpacePose3);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftLvl1ToLeftRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftLvl1Pose);
            waypoints.add(kLeftRocketNearPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftRocketToLoadStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftRocketNearPose);
            waypoints.add(kLoadStationPose2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLeftRocketToLoadStation2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadStationPose4);
            waypoints.add(kLoadStationPose3);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadStationToLeftRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadStationPose3);
            waypoints.add(kLoadStationPose4);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadStationToLeftRocket2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLoadStationPose2);
            waypoints.add(kLeftRocketNearPose2);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath1() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestPose1);
            waypoints.add(kTestPose2);

            // waypoints.add(kTestPose5);
            // waypoints.add(kTestPose6);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kTestPose6);
            waypoints.add(kTestPose7);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftRocketNearPose);
            waypoints.add(kTestPose8);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl1ToRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLvl1Pose);
            waypoints.add(kRightRocketNearPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketToLoadStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightRocketNearPose);
            waypoints.add(kRightLoadStationPose2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getRocketToLoadStation2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLoadStationPose4);
            waypoints.add(kRightLoadStationPose3);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadStationToRocket() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLoadStationPose3);
            waypoints.add(kRightLoadStationPose4);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getLoadStationToRocket2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kRightLoadStationPose2);
            waypoints.add(kRightRocketNearPose2);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        /****************************************************************************************************/

        private Trajectory<TimedState<Pose2dWithCurvature>> getLvl1ToShipBayNear() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLvl1Pose);
            waypoints.add(kShipBayNearPose);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getShipBayNearToStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kShipBayNearPose);
            waypoints.add(kStationPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getShipBayNearToStation2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kStationPose);
            waypoints.add(kStationPose2);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStationToShipBayMid() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kStationPose2);
            // waypoints.add(kShipBayMid);
            waypoints.add(kShipBayMid2);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStationToShipBayMid2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kShipBayMid2);
            waypoints.add(kShipBayMid3);

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getShipBayMidToStation() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kShipBayMid3);
            waypoints.add(kStationPose);

            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(kMaxCentripetalAccel)
            ), kMaxVelocity, kMaxAccel, kMaxVoltage);
        }
    }
}