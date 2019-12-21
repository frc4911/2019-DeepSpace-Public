// created by James Fu on February 8th,2019
// last edited by James Fu on February 16th, 2019

package com.team4911.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.RobotName;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.Robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

////////////////////////////////////////////////////////////////////////////////////////////////
/*

The Collector Subsystem has three components: Two Motors and One Single Solenoid

The main sensor on this subsystem is a Magnetic Encoder that is located on the elbow.
The encoder is used in absolute mode so we apply an offest instead of zeroing it.

Positive input spins the Collector Wheels inward, Negative input spins them outward

Positive input rotates the elbow counter-clockwise, Negative input rotates the elbow clockwise

The Solenoid extends the pistons when set to true, and retracts them when set to false

The Encoder is alligned so that its value reads 0 when the elbow is in its fully stowed position

The Encoder value increases as the elbow spins Counter Clockwise, and decreases as the elbow spins Clockwise

*/
////////////////////////////////////////////////////////////////////////////////////////////////

public class Collector extends Subsystem {

    private static Collector mInstance = new Collector();
    public static CollectorTest mCollectorTest = new CollectorTest();
    private Robot mRobot = null;

    protected PeriodicIO mPeriodicIO;
    // private static final double ELBOW_ENCODER_PPR = 4096;

    //hardware
    private final TalonSRX mRollerTalon;
    private final TalonSRX mElbowTalon;
    private final Solenoid mDeploySolenoid;

    public enum SystemState {
        IDLE,
        STOWING,
        DEPLOYING,
        DEPLOYING_AND_EJECTING,
        BRINGING_UP,
        SELF_TESTING
    }

    public enum WantedState {
        IDLE,
        STOW,
        DEPLOY,
        DEPLOY_AND_EJECT,
        BRING_UP,
        SELF_TEST
    }

    public enum PhysicalSolenoidState {
        EXTENDED,
        RETRACTED
    }

    protected SystemState mSystemState = SystemState.IDLE;
    protected WantedState mWantedState = WantedState.IDLE;

    protected boolean mStateChanged;
    protected PhysicalSolenoidState mSolenoidLastState;

    // DO NOT ZERO THE MAG ENCODER!  APPLY THIS OFFSET!
    // The absolute Mag Encoder has a value of -835 in the stow position
    private static int kEncoderOffset;

    //presets
    private static final int kElbowCollectOffset = 600; // encoder ticks from stowed position
    protected static int kElbowStowedPosition; // Subtract offset to get the encoder position   
    protected static int kElbowCollectPosition; // Subtract offset to get the encoder position
    protected static int kElbowCloseToStowedPosition; // a position just above stowed
    private static final double kRollerIntakeFeed = -1.0; // TODO: arbitrary value, fix this
    private static final double kRollerOutputFeed = 1.0; // TODO: arbitrary value, fix this
    private static final int kElbowPositionPIDTolerance = 20; // TODO: arbitrary value, fix this

    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private boolean mLoggingEnabled = false;

//    private SlotConfiguration mSlotConfigurationElbow;

    private Collector() {
        mPeriodicIO = new PeriodicIO();

        mRollerTalon = TalonSRXFactory.createDefaultTalon(Constants.kCollectorSpinId);

        mElbowTalon = TalonSRXFactory.createDefaultTalon(Constants.kCollectorElbowId);

        TalonSRXUtil.checkError(
                mElbowTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, Constants.kLongCANTimeoutMs),
                "Could not detect collector elbow encoder: ");

        mElbowTalon.setSensorPhase(true);

        TalonSRXUtil.checkError(
                mElbowTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.kLongCANTimeoutMs),
                "Could not set elbow's reverse limit switch source: ");

        mElbowTalon.overrideLimitSwitchesEnable(true);

        mElbowTalon.configPeakOutputForward(0.5);
        mElbowTalon.configPeakOutputReverse(-0.5);

        mDeploySolenoid = new Solenoid(Constants.kCollectorDeployId);

        // force the stowing/homing code to retract solenoid
        mSolenoidLastState = PhysicalSolenoidState.EXTENDED;

        // TODO: create constants for magic numbers
        int encoderValue = mElbowTalon.getSelectedSensorPosition(0);
        if (RobotName.name.equals(Constants.kSilverName)){
            if (encoderValue > -835-100){ 
                kEncoderOffset = -835;
            }
            else {
                kEncoderOffset = -835-4096;
            }
        }
        else {
            System.out.println("collector chrome branch");
            // Chrome
            if (encoderValue > -3900-250){
                kEncoderOffset = -3900;
            }
            else {
                kEncoderOffset = -3900-4096;
            }
        }

        kElbowStowedPosition = kEncoderOffset;
        kElbowCollectPosition = kEncoderOffset + kElbowCollectOffset;
        kElbowCloseToStowedPosition = kElbowStowedPosition + 35; // 35 ticks above stowed position

        // System.out.println("kElbowCollectPosition "+kElbowCollectPosition);
        // System.out.println("kEncoderOffset "+kEncoderOffset);
        // System.out.println("kElbowCollectOffset "+kElbowCollectOffset);
        // System.out.println("kElbowStowedPosition "+kElbowStowedPosition);

        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/COLLECTOR-LOGS.csv", PeriodicIO.class);
    }

    public static Collector getInstance() {
        if (mInstance == null) {
            mInstance = new Collector();
        }
        return mInstance;
    }

    public void registerRobot(Robot robot){
        mRobot = robot;
    }

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            mSystemState = SystemState.IDLE;
            mWantedState = WantedState.IDLE;
            mStateChanged = true;
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Collector.this) {

                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case STOWING:
                        newState = handleStowing(timestamp);
                        break;
                    case DEPLOYING:
                        newState = handleDeploying(false);
                        break;
                    case DEPLOYING_AND_EJECTING:
                        newState = handleDeploying(true);
                        break;
                    case BRINGING_UP:
                        newState = mCollectorTest.handleBringingUp();
                        break;
                    case SELF_TESTING:
                        newState = mCollectorTest.handleSelfTesting(timestamp);
                        break;
                    default:
                        newState = SystemState.STOWING;
                        break;
                }

                if (newState != mSystemState) {
                    System.out.println("Collector State " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            mElbowTalon.set(ControlMode.PercentOutput, 0.0);
            mRollerTalon.set(ControlMode.PercentOutput, 0.0);
        }
    };

    public SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case STOW:
                return SystemState.STOWING;
            case DEPLOY:
                return SystemState.DEPLOYING;
            case DEPLOY_AND_EJECT:
                return SystemState.DEPLOYING_AND_EJECTING;
            case BRING_UP:
                return SystemState.BRINGING_UP;
            case SELF_TEST:
                return SystemState.SELF_TESTING;
            default:
                System.out.println("COLLECTOR STATE ERROR");
                return SystemState.STOWING;
        }
    }

    // stop elbow and roller, move to stow
    private SystemState handleIdle() {
        mPeriodicIO.elbowDemand = 0;
        mPeriodicIO.rollerDemand = 0.0;
        if ((mRobot != null) && mRobot.isTest()){
            mWantedState = WantedState.BRING_UP;
        }
        else {
            mWantedState = WantedState.STOW;
        }
        return defaultStateTransfer();
    }

    private double handleStowingTimeout = 0.0;
    private SystemState handleStowing(double now) {
        if (mStateChanged){
            handleStowingTimeout = now+.25; // short delay to make sure solenoid sufficiently retracted
            mPeriodicIO.solenoidDemand = PhysicalSolenoidState.RETRACTED;
        }

        if (now < handleStowingTimeout){
            mPeriodicIO.solenoidDemand = PhysicalSolenoidState.RETRACTED;
            // drive down slowly if near top
            if (mPeriodicIO.elbowPosition > kElbowCollectPosition-200){
                mPeriodicIO.elbowDemand = -.3;
                mPeriodicIO.rollerDemand = kRollerOutputFeed; // spin out
            }
        }
        // if not at limit and still above the collapsed position then drive down
        else if (!isAtLimit() && (mPeriodicIO.elbowPosition > kElbowCloseToStowedPosition)){
            // TODO: timeout if hung up on ball or bumper
            // full speed down
            mPeriodicIO.rollerDemand = kRollerOutputFeed; // spin out
            mPeriodicIO.elbowDemand = -.5; // collapse
        }
        else {
            // done
            mPeriodicIO.elbowDemand = 0; // stop
            mPeriodicIO.rollerDemand = 0; // stop
        }

        return defaultStateTransfer();
    }

    // deploy, extend elbow, spin roller in
    private SystemState handleDeploying(boolean isEjecting) {
        if (!isEjecting) {
            mPeriodicIO.rollerDemand = kRollerIntakeFeed; // spin in
        } else {
            mPeriodicIO.rollerDemand = -kRollerIntakeFeed; // spin out
        }

        double speed = ((double)(kElbowCollectPosition-mPeriodicIO.elbowPosition+65))/((double)(kElbowCollectPosition-kElbowStowedPosition))*.5;

        mPeriodicIO.elbowDemand = speed;

        // get elbow/arm up past bumper
        if (mPeriodicIO.elbowPosition > kElbowStowedPosition+100){ // TODO: replace 200 with constant
            mPeriodicIO.solenoidDemand = PhysicalSolenoidState.EXTENDED;
        }
        // System.out.println("speed="+speed+" pos="+mPeriodicIO.elbowPosition+" err="+(kElbowCollectPosition-mPeriodicIO.elbowPosition));
        return defaultStateTransfer();
    }

    private boolean isAtLimit() {
        // Since configured normally closed, the actual value read is true when closed and false when tripped
        return !mPeriodicIO.limit_switch;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        // Inputs
        SmartDashboard.putNumber("Collector/Elbow Position", mPeriodicIO.elbowPosition);
        SmartDashboard.putNumber("Collector/Adjusted Elbow Position", mPeriodicIO.elbowPosition - kEncoderOffset);
        SmartDashboard.putBoolean("Collector/At limit", isAtLimit());
        SmartDashboard.putNumber("Collector/Elbow current", mPeriodicIO.elbow_current);
        SmartDashboard.putNumber("Collector/Roller current", mPeriodicIO.roller_current);

        // Outputs
        // SmartDashboard.putNumber("Collector/Elbow Speed", mPeriodicIO.elbowDemand);
        // SmartDashboard.putNumber("Collector/Roller Speed", mPeriodicIO.rollerDemand);
        // SmartDashboard.putString("Collector/Solenoid", mPeriodicIO.solenoidDemand.toString());
        // SmartDashboard.putString("Collector/Solenoid State", mSolenoidLastState.toString());
        SmartDashboard.putString("Collector/State", mSystemState.toString());
    }

    @Override
    public void stop() {
        mElbowTalon.set(ControlMode.PercentOutput, 0.0);
        mRollerTalon.set(ControlMode.PercentOutput, 0.0);
        stopLogging();
    //    mDeploySolenoid.set(kSolenoidRetracted);
    }

    public synchronized void startLogging() { mLoggingEnabled = true; }

    public synchronized void stopLogging() {
        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.flush();
        }

        mLoggingEnabled = false;
    }

    @Override
    public void writeToLog() {
        if (mLoggingEnabled  && mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.elbowPosition = mElbowTalon.getSelectedSensorPosition(0);
        mPeriodicIO.limit_switch = mElbowTalon.getSensorCollection().isRevLimitSwitchClosed();
        mPeriodicIO.elbow_current = mElbowTalon.getOutputCurrent();
        mPeriodicIO.roller_current = mRollerTalon.getOutputCurrent();

       if (isAtLimit()) {
           kEncoderOffset = mPeriodicIO.elbowPosition;
           kElbowStowedPosition = kEncoderOffset;
           kElbowCollectPosition = kEncoderOffset + kElbowCollectOffset;
           kElbowCloseToStowedPosition = kElbowStowedPosition + 35; // 35 ticks above stowed position
       }

        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mElbowTalon.set(ControlMode.PercentOutput, mPeriodicIO.elbowDemand);

        mRollerTalon.set(ControlMode.PercentOutput, mPeriodicIO.rollerDemand);

        if (mSolenoidLastState != mPeriodicIO.solenoidDemand) {
            mDeploySolenoid.set(mPeriodicIO.solenoidDemand == PhysicalSolenoidState.EXTENDED);
            mSolenoidLastState = mPeriodicIO.solenoidDemand;
        }
    }

    public static class PeriodicIO {
        //Inputs
        public int elbowPosition;
        public boolean limit_switch;
        public double elbow_current;
        public double roller_current;

        //outputs
        public double elbowDemand;
        public double rollerDemand;
        public PhysicalSolenoidState solenoidDemand = PhysicalSolenoidState.RETRACTED;
    }

    // TODO: change this to read limit switch only if end up using
    // public synchronized boolean isStowed() {
    //     return ((mSolenoidLastState == PhysicalSolenoidState.RETRACTED) &&
    //             (mPeriodicIO.elbowPosition > kElbowCloseToStowedPosition));
    // }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    public synchronized boolean isDeployed() { return mPeriodicIO.solenoidDemand == PhysicalSolenoidState.EXTENDED; }
}