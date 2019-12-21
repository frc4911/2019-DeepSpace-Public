package com.team4911.frc2019.subsystems;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.sensors.lidar.VL53L0X;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.Robot;

import com.team4911.frc2019.states.TimedLEDState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
The HatchIntake Subsystem has two single solenoids that have two state [true, false]
The true state pushes the piston out
The false state retracts the piston back

The ldar is a sensor hooked up to the i2c bus
The lidar reports back a distance in millimeters?
If the lidar reports a value that is in range of the "in range" constant,
then the solenoids can fire/shoot, otherwise it is unable to fire/shoot
*/
public class HatchIntake extends Subsystem {

    private static HatchIntake sInstance = null;
    public static HatchTest mHatchTest = new HatchTest();
    private Robot mRobot = null;

    public static HatchIntake getInstance() {
        if (sInstance == null) {
            sInstance = new HatchIntake();
        }

        return sInstance;
    }

    // Hardware
    private final Solenoid mDeployer, mEjector;
    private VL53L0X mLidar;
//    private MovingAverage readings = new MovingAverage(5);

    private final double kInRangeOfTarget = 10 * 25.4;  // 10 inches in millimeters
    private final double kEjectingTimeout = 0.1;
    private final double kSequencingTimeout = 0.1;
    private final int kLidarThresholdCount = 3;
    static AnalogInput pressureSensor = new AnalogInput(Constants.kPressureId);

    public enum SystemState {
        IDLE,
        STOWING,
        OPEN_BEAK,
        OPEN_AND_EJECTING,
        BRINGING_UP,
        SELF_TESTING
    }

    public enum WantedState {
        IDLE,
        STOW,
        OPEN,
        OPEN_AND_EJECT,
        BRING_UP,
        SELF_TEST
    }

    private enum EjectAndOpenState {
        WAITING,
        OPENING,
        EJECTING
    }

    public enum BeakSolenoidState {
        OPEN,
        CLOSE
    }

    public enum EjectSolenoidState {
        RETRACT,
        EJECT
    }

    protected SystemState mSystemState = SystemState.IDLE;
    protected WantedState mWantedState = WantedState.IDLE;
    private EjectAndOpenState mEAndOState = EjectAndOpenState.WAITING;
    private BeakSolenoidState mBeakState = BeakSolenoidState.CLOSE;
    private EjectSolenoidState mEjectState = EjectSolenoidState.RETRACT;
    protected PeriodicIO mPeriodicIO;
    private double mEjectTimeout;
    private double mSequenceTimeout;
    protected boolean mStateChanged;
    private boolean mDAndEStateChanged;
    private boolean mManualOverride = false;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    boolean mLoggingEnabled = false;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (HatchIntake.this) {
                mSystemState = SystemState.IDLE;
                mWantedState = WantedState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (HatchIntake.this) {
                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case STOWING:
                        newState = handleStowing();
                        break;
                    case OPEN_BEAK:
                        newState = handleOpening();
                        break;
                    case OPEN_AND_EJECTING:
                        newState = handleOpeningAndEjecting(timestamp);
                        break;
                    case BRINGING_UP:
                        newState = mHatchTest.handleBringingUp();
                        break;
                    case SELF_TESTING:
                        newState = mHatchTest.handleSelfTesting(timestamp);
                        break;
                    default:
                        newState = SystemState.STOWING;
                }

                if (newState != mSystemState) {
                    System.out.println("Hatch intake state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    public HatchIntake() {
        mPeriodicIO = new PeriodicIO();
        mDeployer = new Solenoid(0, Constants.kOpenBeakId);
        mEjector = new Solenoid(0, Constants.kHatchDeployId);

        //  try {
        //      mLidar = new VL53L0X(I2C.Port.kOnboard, 10);
        //  } catch (RuntimeException ex ) {
        //      DriverStation.reportError("Error instantiating VL53L0X:  " + ex.getMessage(), true);
        //  }

        // Start the hatch intake in non-deployed and retracted position.  Set true to force a state change.
        mBeakState = BeakSolenoidState.OPEN;
        mEjectState = EjectSolenoidState.EJECT;

        mPeriodicIO.beak_demand = mBeakState;
        mPeriodicIO.eject_demand = mEjectState;

        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/HATCH-LOGS.csv", HatchIntake.PeriodicIO.class);
    }

    public void registerRobot(Robot robot) {
        mRobot = robot;
    }

    protected SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case STOW:
                return SystemState.STOWING;
            case OPEN:
                return SystemState.OPEN_BEAK;
            case OPEN_AND_EJECT:
                return SystemState.OPEN_AND_EJECTING;
            case BRING_UP:
                return SystemState.BRINGING_UP;
            case SELF_TEST:
                return SystemState.SELF_TESTING;
            default:
                return SystemState.STOWING;
        }
    }

    private SystemState handleIdle() {
        mPeriodicIO.beak_demand = BeakSolenoidState.CLOSE;
        mPeriodicIO.eject_demand = EjectSolenoidState.RETRACT;

        if ((mRobot != null) && (mRobot.isTest())) {
            mWantedState = WantedState.BRING_UP;
        }
        return defaultStateTransfer();
    }

    private SystemState handleStowing() {
        mPeriodicIO.beak_demand = BeakSolenoidState.CLOSE;
        mPeriodicIO.eject_demand = EjectSolenoidState.RETRACT;

        return defaultStateTransfer();
    }

    private SystemState handleOpening() {
        mPeriodicIO.beak_demand = BeakSolenoidState.OPEN;
        return defaultStateTransfer();
    }

    private SystemState handleOpeningAndEjecting(double timestamp) {
        if (mStateChanged) {
            mEAndOState = EjectAndOpenState.WAITING;
            mDAndEStateChanged = true;
        }

        EjectAndOpenState newState;
        switch (mEAndOState) {
            case WAITING:
                newState = handleOAndEWaiting();
                break;
            case EJECTING:
                newState = handleOAndEEjecting(timestamp);
                break;
            case OPENING:
                newState = handleOAndEOpening(timestamp);
                break;
            default:
                newState = EjectAndOpenState.WAITING;
        }
        if (mEAndOState != newState) {
            mEAndOState = newState;
            mDAndEStateChanged = true;
        } else {
            mDAndEStateChanged = false;
        }
        return defaultStateTransfer();
    }

    private EjectAndOpenState handleOAndEWaiting() {
        if (mPeriodicIO.isReady || isManualOverride()) {
            return EjectAndOpenState.EJECTING;
        }

        return EjectAndOpenState.WAITING;
    }

    private EjectAndOpenState handleOAndEEjecting(double timestamp) {
        if (mDAndEStateChanged) {
            mSequenceTimeout = timestamp + kSequencingTimeout;
        } else if (timestamp >= mSequenceTimeout) {
            return EjectAndOpenState.OPENING;
        }

        mPeriodicIO.eject_demand = EjectSolenoidState.EJECT;

        return EjectAndOpenState.EJECTING;
    }

    private EjectAndOpenState handleOAndEOpening(double timestamp) {
        if (mDAndEStateChanged) {
            mEjectTimeout = timestamp + kEjectingTimeout;
        } else if (timestamp >= mEjectTimeout) {
            mPeriodicIO.eject_demand = EjectSolenoidState.RETRACT;
        }

        mPeriodicIO.beak_demand = BeakSolenoidState.OPEN;

        return EjectAndOpenState.OPENING;
    }

    public synchronized double getLidarRange() {
        return mPeriodicIO.lidar_value;
    }

    protected synchronized void setLidarRange(double range) {
        mPeriodicIO.lidar_value = range;
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    public synchronized void setManualOverride(boolean manualOverride) {
        mManualOverride = manualOverride;
    }

    public synchronized boolean isManualOverride() {
        return mManualOverride;
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
        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public void readPeriodicInputs() {
        // TODO: Is this test code?
        // 1.5 psi drop for each beak open (3 psi for open and close)
        // 2 psi drop for each piston extend (4 psi for open and close)
        // Analog = 720 when dial = 20 psi
        // Analog = 1415 when dial = 65 psi
        // psi = analog*.065 - 26.6 (on Silver)
        mPeriodicIO.pneumaticPressure = (int)(((double)pressureSensor.getValue())*0.065-26.6);

        // mPeriodicIO.lidar_value = VL53L0X.INVALID_RANGE ;
        // if ((mLidar.isInitialized() && mLidar.isConnected()) || mManualOverride) {
        //     mPeriodicIO.lidar_value = mLidar.getRange();
        // }

        // TODO:  Restore above code when Lidar is fully functional
//        mPeriodicIO.lidar_value = 0;
//        if (mPeriodicIO.lidar_value < kInRangeOfTarget) {
//            if (mPeriodicIO.lidar_threshold_count++ >= kLidarThresholdCount) {
                mPeriodicIO.isReady = true;
//            }
//        } else {
//            mPeriodicIO.lidar_threshold_count = 0;
//            mPeriodicIO.isReady = false;
//        }

        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        // Try to avoid hitting CAN/JNI wrapper.
        if (mPeriodicIO.beak_demand != mBeakState) {
            mBeakState = mPeriodicIO.beak_demand;
            mDeployer.set(mBeakState == BeakSolenoidState.OPEN);    
        }
        
        // Try to avoid hitting CAN/JNI wrapper.
        if (mPeriodicIO.eject_demand != mEjectState) {
            mEjectState = mPeriodicIO.eject_demand;
            mEjector.set(mEjectState == EjectSolenoidState.EJECT);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Hatch/SystemState", mSystemState.toString());
        // SmartDashboard.putBoolean("Hatch/In Range of Rocket", mPeriodicIO.isReady);
        SmartDashboard.putNumber("Hatch/Lidar", mPeriodicIO.lidar_value);
        SmartDashboard.putString("Hatch/Ejector Set", mPeriodicIO.eject_demand.toString());
        SmartDashboard.putString("Hatch/Beak Set", mPeriodicIO.beak_demand.toString());
        // SmartDashboard.putString("Hatch/Open and Eject State", mEAndOState.toString());
        SmartDashboard.putNumber("Hatch/Pneumatic Pressure", mPeriodicIO.pneumaticPressure);
//        SmartDashboard.putBoolean("Hatch/Lidar isInitialized", mLidar.isInitialized());
//        SmartDashboard.putBoolean("Hatch/Lidar isConnected", mLidar.isConnected());
//        SmartDashboard.putNumber("Hatch/Lidar updates", mLidar.getUpdateCount());
    }

    @Override
    public void stop() {
        stopLogging();
    }

    @Override
    public void zeroSensors() {
        // No-op.
    }

    public synchronized boolean isBeakDown() { return mPeriodicIO.beak_demand ==  BeakSolenoidState.OPEN; }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    public static class PeriodicIO {
        //INPUTS
        public double lidar_value;
        public boolean isReady;
        public double pneumaticPressure;

        // OUTPUTS
        public BeakSolenoidState beak_demand=BeakSolenoidState.CLOSE;
        public EjectSolenoidState eject_demand=EjectSolenoidState.RETRACT;
    }
}
