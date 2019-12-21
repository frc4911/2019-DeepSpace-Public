package com.team4911.frc2019.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.Robot;
import com.team4911.frc2019.RobotState;
import com.team4911.frc2019.planners.DriveMotionPlanner;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.RobotName;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {

    private static final int kLowGearVelocityControlSlot = 0;
    private static final int kHighGearVelocityControlSlot = 1;
    private static final double DRIVE_ENCODER_PPR = 4821.6; // 3072
    private static Drive mInstance = new Drive();
    public static DriveTest mDriveTest = new DriveTest();
    private Robot mRobot = null;
    private boolean mThreeMotors = true;
    private boolean mUsePigeon = true;

    // Hardware
    protected final TalonSRX mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA;
    protected TalonSRX mLeftSlaveB=null, mRightSlaveB=null;
  
    private Solenoid mShifter = null;

    // Control states
    private PigeonIMU mPigeon;
    private AHRS mNavXBoard = null;

    // Hardware states
    protected PeriodicIO mPeriodicIO;
    private boolean mAutoShift;
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private DriveMotionPlanner mMotionPlanner;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private boolean mOverrideTrajectory = false;

    private double ypr_deg[] = {1, 1, 1};

	public enum SystemState {
		IDLE, // dpes nothing
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
        SELF_TESTING, // predefined drive system exersize
        BRINGING_UP // simple functional tests
	}

	public enum WantedState {
		IDLE, 
        OPEN_LOOP,
        PATH_FOLLOW,
        SELF_TEST,
        BRING_UP
	}

	private SystemState mSystemState = SystemState.IDLE;
	protected WantedState mWantedState = WantedState.IDLE;
	protected boolean mStateChanged;

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private boolean mLoggingEnabled = false;


    public enum DriveTalon {
        MASTER_LEFT,
        MASTER_RIGHT,
        SLAVE_A_LEFT,
        SLAVE_A_RIGHT,
        SLAVE_B_LEFT,
        SLAVE_B_RIGHT,
        ALL_LEFT,
        ALL_RIGHT
    }

    // used to check test mode
    public void registerRobot(Robot robot){
        mRobot = robot;
    }

    protected SystemState defaultStateTransfer() {
		switch (mWantedState) {
			case IDLE:
				return SystemState.IDLE;
            case OPEN_LOOP:
                return SystemState.OPEN_LOOP;
            case PATH_FOLLOW:
                return SystemState.PATH_FOLLOWING;
            case SELF_TEST:
				return SystemState.SELF_TESTING;
            case BRING_UP:
                return SystemState.BRINGING_UP;
			default:
				System.out.println("DRIVE STATE ERROR");
				return SystemState.IDLE;
		}
	}

    private final Loop mLoop = new Loop() {
        
        @Override
        public void onStart(double timestamp) {
            synchronized (Drive.this) {
                mSystemState = SystemState.IDLE;
                mWantedState = WantedState.IDLE;
                setOpenLoop(new DriveSignal(0.0, 0.0));
                setBrakeMode(false);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Drive.this) {
				SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case OPEN_LOOP:
                        newState = handleOpenLoop();
                        break;
                    case PATH_FOLLOWING:
                        newState = updatePathFollower();
                        break;
                    case SELF_TESTING:
                        newState = mDriveTest.handleSelfTesting(timestamp);
                        break;
                    case BRINGING_UP:
                        newState = mDriveTest.handleBringUp();
                        break;
                    default:
                        newState = SystemState.IDLE;
                        System.out.println("Unexpected drive control state: " + mSystemState);
                        break;
                }

                if (newState != mSystemState){
					System.out.println( "Drive State " + mSystemState + " to " + newState);
					mSystemState = newState;
					mStateChanged = true;
				} else {
					mStateChanged = false;
				}

            }
        }

        @Override
        public void onStop(double timestamp) {
            mRightMaster.set(ControlMode.PercentOutput, 0.0);
            mLeftMaster.set(ControlMode.PercentOutput, 0.0);
        }
    };

    private SystemState handleIdle() {
        if(mStateChanged) {
            restoreFollowers();
            mPeriodicIO.right_demand = 0.0;
            mPeriodicIO.left_demand = 0.0;
        }
        
        if ((mRobot != null) && (mRobot.isTest())){
            mWantedState = WantedState.BRING_UP;
        }
        return defaultStateTransfer();
    }    

    private SystemState handleOpenLoop(){
        return defaultStateTransfer();
    }

    private void configureMaster(TalonSRX talon, boolean left) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);

        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice
                .QuadEncoder, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }
        
        talon.setInverted(left);
        talon.setSensorPhase(false);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    private void restoreFollowers() {
        mLeftSlaveA.set(ControlMode.Follower, Constants.kDriveLeftMasterId);
        mRightSlaveA.set(ControlMode.Follower, Constants.kDriveRightMasterId);

        if (mThreeMotors){
            mLeftSlaveB.set(ControlMode.Follower, Constants.kDriveLeftMasterId);
            mRightSlaveB.set(ControlMode.Follower, Constants.kDriveRightMasterId);
        }
    }

    private Drive() {

        if (RobotName.name.equals(Constants.kWeebleName)){
            mThreeMotors = false;
            mUsePigeon = false;
        } else if (RobotName.name.equals(Constants.kSilverName)) {
            mUsePigeon = false;
        }

        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode.
        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kDriveLeftMasterId);
        configureMaster(mLeftMaster, true);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kDriveLeftSlaveAId,
                Constants.kDriveLeftMasterId);
        mLeftSlaveA.setInverted(true);

        if (mThreeMotors){
            mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kDriveLeftSlaveBId,
            Constants.kDriveLeftMasterId);
            mLeftSlaveB.setInverted(true);
        }

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kDriveRightMasterId);
        configureMaster(mRightMaster, false);

        mRightSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kDriveRightSlaveAId,
                Constants.kDriveRightMasterId);
        mRightSlaveA.setInverted(false);

        if (mThreeMotors){
            mRightSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kDriveRightSlaveBId,
                Constants.kDriveRightMasterId);
            mRightSlaveB.setInverted(false);
        }

        mShifter = new Solenoid(0, Constants.kDriveShifterId);
       
        reloadGains();

        if (mUsePigeon){
            mPigeon = new PigeonIMU(mLeftSlaveA);
        }
        else {
            mNavXBoard = new AHRS(SPI.Port.kMXP, (byte) 200);
        }

        mRightSlaveA.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

//        mIsHighGear = true;
        mIsHighGear = false;
//        setHighGear(false);

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();

        mCSVWriter = new ReflectingCSVWriter<PeriodicIO>("/home/lvuser/DRIVE-LOGS.csv", Drive.PeriodicIO.class);
    }

    public static Drive getInstance() {
        return mInstance;
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * DRIVE_ENCODER_PPR / 10.0;
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mSystemState != SystemState.OPEN_LOOP) {
            restoreFollowers();
            setBrakeMode(false);

            // System.out.println("Switching to open loop");
            // System.out.println(signal);
            mWantedState = WantedState.OPEN_LOOP;
            mLeftMaster.configNeutralDeadband(0.04, 0);
            mRightMaster.configNeutralDeadband(0.04, 0);
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mSystemState != SystemState.PATH_FOLLOWING) {
            // We entered a velocity control state.
            restoreFollowers();
            setBrakeMode(true);
            mAutoShift = false;
            mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            mWantedState = WantedState.PATH_FOLLOW;
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mWantedState = WantedState.PATH_FOLLOW;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mSystemState != SystemState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        SmartDashboard.putBoolean("High Gear", wantsHighGear);
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            mShifter.set(wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);

            if (mThreeMotors){
                mRightSlaveB.setNeutralMode(mode);
                mLeftSlaveB.setNeutralMode(mode);
            }
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        if (mUsePigeon){
            mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        }
        else {
            mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(-mNavXBoard.getFusedHeading()).inverse());
        }
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    @Override
    public synchronized void stop() {
        stopLogging();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Drive/SystemState", mSystemState.toString());
        SmartDashboard.putNumber("Drive/Right Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Drive/Left Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Drive/Right Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Drive/Left Distance", mPeriodicIO.left_distance);
        // SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        // SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        SmartDashboard.putNumber("Drive/x err", mPeriodicIO.error.getTranslation().x());
        SmartDashboard.putNumber("Drive/y err", mPeriodicIO.error.getTranslation().y());
        SmartDashboard.putNumber("Drive/theta err", mPeriodicIO.error.getRotation().getDegrees());
        if (getHeading() != null) {
            SmartDashboard.putNumber("Drive/Gyro Heading", getHeading().getDegrees());
        }
    }

    public synchronized void resetEncoders() {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
        mAutoShift = true;
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private SystemState updatePathFollower() {
        if (mSystemState == SystemState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }

        return defaultStateTransfer();
    }

    private void handleAutoShift() {
        final double linear_velocity = Math.abs(getLinearVelocity());
        final double angular_velocity = Math.abs(getAngularVelocity());
        if (mIsHighGear && linear_velocity < Constants.kDriveDownShiftVelocity && angular_velocity < Constants
                .kDriveDownShiftAngularVelocity) {
            setHighGear(false);
        } else if (!mIsHighGear && linear_velocity > Constants.kDriveUpShiftVelocity) {
            setHighGear(true);
        }
    }

    public synchronized void reloadGains() {
        mLeftMaster.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);

        mRightMaster.config_kP(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKp, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kI(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKi, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kD(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKd, Constants.kLongCANTimeoutMs);
        mRightMaster.config_kF(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityKf, Constants.kLongCANTimeoutMs);
        mRightMaster.config_IntegralZone(kLowGearVelocityControlSlot, Constants.kDriveLowGearVelocityIZone, Constants.kLongCANTimeoutMs);
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    @Override
    public void writeToLog() {
        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
        if (mUsePigeon){
            mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);
        }
        else {
            mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(-mNavXBoard.getFusedHeading()).rotateBy(mGyroOffset);
        }
        mPeriodicIO.left_distance += ((mPeriodicIO.left_position_ticks - prevLeftTicks) / DRIVE_ENCODER_PPR) * Math.PI * Constants.kDriveWheelDiameterInches;
        mPeriodicIO.right_distance += ((mPeriodicIO.right_position_ticks - prevRightTicks) / DRIVE_ENCODER_PPR) * Math.PI * Constants.kDriveWheelDiameterInches;

        mPeriodicIO.leftMaster_current = mLeftMaster.getOutputCurrent();
        mPeriodicIO.leftSlaveA_current = mLeftSlaveA.getOutputCurrent();
        mPeriodicIO.rightMaster_current = mRightMaster.getOutputCurrent();
        mPeriodicIO.rightSlaveA_current = mRightSlaveA.getOutputCurrent();

        if (mThreeMotors){
            mPeriodicIO.leftSlaveB_current = mLeftSlaveB.getOutputCurrent();
            mPeriodicIO.rightSlaveB_current = mRightSlaveB.getOutputCurrent();
        }
        else {
            mPeriodicIO.leftSlaveB_current = 0;
            mPeriodicIO.rightSlaveB_current = 0;
        }

        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }

        if (mUsePigeon){
            mPigeon.getYawPitchRoll(ypr_deg);

            // SmartDashboard.putNumber("Yaw", ypr_deg[0]);
            // SmartDashboard.putNumber("Pitch", ypr_deg[1]);
            // SmartDashboard.putNumber("Roll", ypr_deg[2]);
        }
        else {
            SmartDashboard.putNumber("Yaw", mNavXBoard.getYaw());
            SmartDashboard.putNumber("Pitch", mNavXBoard.getPitch());
            SmartDashboard.putNumber("Roll", mNavXBoard.getRoll());
        }
        // System.out.println("control state: " + mSystemState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mSystemState == SystemState.OPEN_LOOP || mSystemState == SystemState.IDLE) {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        } 
        else if (mSystemState == SystemState.PATH_FOLLOWING) {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / 1023.0);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / 1023.0);
        }
        else if (mSystemState == SystemState.BRINGING_UP || mSystemState == SystemState.SELF_TESTING){
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        }
    }

    public synchronized void startLogging() { mLoggingEnabled = true; }

    public synchronized void stopLogging() {
        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.flush();
        }

        mLoggingEnabled = false;
    }

    public enum ShifterState {
        FORCE_LOW_GEAR,
        FORCE_HIGH_GEAR,
        AUTO_SHIFT
    }

    public static class PeriodicIO {
        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        public double leftMaster_current;
        public double leftSlaveA_current;
        public double leftSlaveB_current;
        public double rightMaster_current;
        public double rightSlaveA_current;
        public double rightSlaveB_current;

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}