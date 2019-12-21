package com.team4911.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
The shooter subsystem has one motor that outputs three speeds [-0.3, 0, 0.3]
The intaking output (-0.8) rotates the motor backwards
The shooting output (0.8) rotates the motor forwards
The holding output (0.0) doesn't rotate the motor

The beam break is a digital input that returns a true or false
The beam break returns true if the beam is not broken
The beam break return false if the beam is broken (by the cargo)
When the beam break returns false the shooter motor stops intaking
*/
public class Shooter extends Subsystem {

    private static Shooter sInstance = null;
    public static ShooterTest mShooterTest = new ShooterTest();
    private Robot mRobot = null;

    public static Shooter getInstance() {
        if(sInstance == null) {
            sInstance = new Shooter();
        }

        return sInstance;
    }

    // Hardware
    protected final TalonSRX mTalon;
    private final DigitalInput mBeamBreak;

    protected final double kShootSpeed = 0.8;
    protected final double kIntakeSpeed = -0.8;

    public enum SystemState {
        IDLE,
        HOLDING,
        INTAKING,
        SHOOTING,
        SELF_TESTING,
        BRINGING_UP
    }

    public enum WantedState {
        IDLE,
        HOLD,
        INTAKE,
        SHOOT,
        SELF_TEST,
        BRING_UP
    }

    protected SystemState mSystemState = SystemState.IDLE;
    protected WantedState mWantedState = WantedState.IDLE;
    protected PeriodicIO mPeriodicIO;
    protected boolean mStateChanged;
    private boolean mManualOverride = false;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private boolean mLoggingEnabled = false;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized(Shooter.this) {
                mSystemState = SystemState.IDLE;
                mWantedState = WantedState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Shooter.this) {
                SystemState newState;
                switch(mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    case INTAKING:
                        newState = handleIntaking();
                        break;
                    case SHOOTING:
                        newState = handleShooting();
                        break;
                    case SELF_TESTING:
                        newState = mShooterTest.handleSelfTesting(timestamp);
                        break;
                    case BRINGING_UP:
                        newState = mShooterTest.handleBringUp();
                        break;
                    default:
                        newState = SystemState.HOLDING;
                }

                if (newState != mSystemState) {
                    System.out.println("Shooter state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            mTalon.set(ControlMode.PercentOutput, 0);
        }
    };

    public Shooter() {
        mPeriodicIO = new PeriodicIO();
        mTalon = TalonSRXFactory.createDefaultTalon(Constants.kShooterId);
        mBeamBreak = new DigitalInput(Constants.kShooterBeamBreakId);
        setNeutralMode(NeutralMode.Brake);

        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/SHOOTER-LOGS.csv", Shooter.PeriodicIO.class);
    }

    public void registerRobot(Robot robot){
        mRobot = robot;
    }

    private void setNeutralMode(NeutralMode mode) {
        mTalon.setNeutralMode(mode);
    }

    protected SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case HOLD:
                return SystemState.HOLDING;
            case INTAKE:
                return SystemState.INTAKING;
            case SHOOT:
                return SystemState.SHOOTING;
            case BRING_UP:
                return SystemState.BRINGING_UP;
            case SELF_TEST:
                return SystemState.SELF_TESTING;
            default:
                return SystemState.HOLDING;
        }
    }

    private SystemState handleIdle() {
        mPeriodicIO.demand = 0.0;
        if ((mRobot != null) && (mRobot.isTest())){
            mWantedState = WantedState.BRING_UP;
        }
        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        mPeriodicIO.demand = 0.0;
        return defaultStateTransfer();
    }

    private SystemState handleIntaking() {
        if (!hasCargo() || isManualOverride()) {
            mPeriodicIO.demand = kIntakeSpeed;
        } else {
            mWantedState = WantedState.HOLD;
        }
        return defaultStateTransfer();
    }

    private SystemState handleShooting() {
        mPeriodicIO.demand = kShootSpeed;
        return defaultStateTransfer();
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


    public synchronized boolean hasCargo() {
        return !mPeriodicIO.beamBreakIsOn;
    }

    public synchronized void startLogging() { mLoggingEnabled = true ; }

    public synchronized void stopLogging() {
        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.flush();
        }
        mLoggingEnabled = false;
    }

    @Override
    public void writeToLog() {
        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.beamBreakIsOn = mBeamBreak.get();
        mPeriodicIO.isManualOverride = isManualOverride();
        mPeriodicIO.current_draw = mTalon.getOutputCurrent();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        mTalon.set(ControlMode.PercentOutput, mPeriodicIO.demand);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Shooter/Beam Break", mPeriodicIO.beamBreakIsOn);
        SmartDashboard.putNumber("Shooter/Speed", mPeriodicIO.demand);
        SmartDashboard.putString("Shooter/SystemState", mSystemState.toString());
        SmartDashboard.putNumber("Shooter/Talon current", mPeriodicIO.current_draw);
        
        // SmartDashboard.putBoolean("Shooter/Manual Override", isManualOverride());
    }

    @Override
    public void stop() {
        // Always disable motors on stop
        mTalon.set(ControlMode.PercentOutput, 0);
        stopLogging();
    }

    @Override
    public void zeroSensors() {
        // No-op.
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean beamBreakIsOn;
        public boolean isManualOverride;
        public double current_draw;

        //OUTPUTS
        public double demand;
    }
}
