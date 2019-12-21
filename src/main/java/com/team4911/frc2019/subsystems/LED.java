package com.team4911.frc2019.subsystems;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.states.LEDState;
import com.team4911.frc2019.states.TimedLEDState;

import edu.wpi.first.wpilibj.Timer;

/**
 * A subsystem for LED managing state and effects.
 */
public class LED extends Subsystem {
    private static final double kClimbingBlinkDuration = 0.5; // In sec
    private static final double kWantsCargoBlinkDuration = 0.075; // In sec
    private static final double kFaultBlinkDuration = 0.25; // In sec

    private static LED mInstance;

    private LEDCanifier mLEDCanifier;
    private SystemState mSystemState = SystemState.DISPLAYING_BEAK_DOWN;
    private WantedAction mWantedAction = WantedAction.DISPLAY_BEAK_DOWN;

    // List subsystems that have LED state

    // private Wrist mWrist;
    // private Elevator mElevator;

    private boolean mFaultsEnabled = false;
    private boolean mErrorCondition = false;
    private boolean mBlinking = false;

    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);
    private TimedLEDState mActiveLEDState = TimedLEDState.StaticLEDState.kStaticOff;

    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }

        return mInstance;
    }

    private LED() {
        mLEDCanifier = LEDCanifier.getInstance();
    }

   public synchronized void setWantedAction(WantedAction wantedAction, boolean isBlinking) {
        mWantedAction = wantedAction;
        mBlinking = isBlinking;
    }

    private Loop loop = new Loop() {
            double stateStartTime;

            @Override
            public void onStart(double timestamp) {
                stateStartTime = timestamp;
                // mWrist = Wrist.getInstance();
                // mElevator = Elevator.getInstance();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {
                    SystemState newState = getStateTransition();

                    if (mSystemState != newState) {
                        System.out.println(timestamp + ": LED changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        stateStartTime = timestamp;
                    }

                    double timeInState = timestamp - stateStartTime;

                    switch (mSystemState) {
                        case DISPLAYING_FAULT:
                            setFaultLEDCommand(timeInState);
                            break;
                        case DISPLAYING_CLIMB:
                            setClimbLEDCommand(timeInState);
                            break;
                        case DISPLAYING_BEAK_DOWN:
                        case DISPLAYING_HAS_CARGO:
                        case DISPLAYING_COLLECTOR_DEPLOYED:
                        case DISPLAYING_DRIVE:
                        case DISPLAYING_AUTO_START:
                            mActiveLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
                            break;
                        default:
                            System.out.println("Fell through on LED commands: " + mSystemState);
                            break;
                    }
                    
                    mLEDCanifier.setLEDColor(mDesiredLEDState.red, mDesiredLEDState.green, mDesiredLEDState.blue);
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        };
    

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    private void setFaultLEDCommand(double timeInState) {
        // Blink red.
        if ((int) (timeInState / kFaultBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kFault);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private void setClimbLEDCommand(double timeInState) {
        // Blink orange
        if ((int) (timeInState / kClimbingBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kClimbing);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private SystemState getStateTransition() {
        // if (mFaultsEnabled && (!mWrist.hasBeenZeroed() ||
        // !mElevator.hasBeenZeroed())) {
        // return SystemState.DISPLAYING_FAULT;
        // }
        if (mErrorCondition) {
            return SystemState.DISPLAYING_FAULT;
        }

        switch (mWantedAction) {
            case DISPLAY_CLIMB:
                return SystemState.DISPLAYING_CLIMB;
            case DISPLAY_BEAK_DOWN:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kBeakDown : TimedLEDState.StaticLEDState.kBeakDown;
                return SystemState.DISPLAYING_BEAK_DOWN;
            case DISPLAY_HAS_CARGO:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kHasCargo : TimedLEDState.StaticLEDState.kHasCargo;
                return SystemState.DISPLAYING_HAS_CARGO;
            case DISPLAY_COLLECTOR_DEPLOYED:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kCollectorDeployed: TimedLEDState.StaticLEDState.kCollectorDeployed;
                return SystemState.DISPLAYING_COLLECTOR_DEPLOYED;
            case  DISPLAY_DRIVE:
                mActiveLEDState = mBlinking ? TimedLEDState.BlinkingLEDState.kDrive: TimedLEDState.StaticLEDState.kDrive;
                return SystemState.DISPLAYING_DRIVE;
            case DISPLAY_AUTO_START:
                mActiveLEDState = TimedLEDState.StaticLEDState.kAutoStart;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.DISPLAYING_BEAK_DOWN;
        }
    }

    public synchronized void setEnableFaults(boolean enable) {
        mFaultsEnabled = enable;
    }

    boolean testRunOnce = false;
    @Override
    public boolean checkSystem() {
        if (testRunOnce) {
            return true;
        }
//
//        System.out.println("Climb - blink orange for 0.5s");
//        this.setWantedAction(WantedAction.DISPLAY_CLIMB);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking - solid green for 0.5s");
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kIntaking);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking (has cargo) - blinking blue for 0.075s");
//        setIntakeLEDState(TimedLEDState.BlinkingLEDState.kHasCargo);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test intake sequence:  Intaking -> Blinking Has Cargo -> Solid Has Cargo
//        System.out.println("Intaking (has cargo) - solid blue");
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kHasCargo);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        // Test wants Cargo sequence
//        System.out.println("Wants Cargo - blinking orange");
//        this.setWantedAction(WantedAction.DISPLAY_HAS_CARGO);
//        runLoop(3.0);
//
//        // Test Fault LED
//        System.out.println("Wants Cargo - blinking red");
//        mErrorCondition = true;
//        runLoop(3.0);
//        mErrorCondition = false;
//
//        setIntakeLEDState(TimedLEDState.StaticLEDState.kStaticOff);
//        this.setWantedAction(WantedAction.DISPLAY_BEAK_DOWN);
//        runLoop(3.0);
//
//        mDesiredLEDState.blue = 0;
//        mDesiredLEDState.red= 0;
//        mDesiredLEDState.green = 0;
//
//        testRunOnce = true;
        return true;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }

    public enum WantedAction {
        DISPLAY_CLIMB,
        DISPLAY_BEAK_DOWN,
        DISPLAY_HAS_CARGO,
        DISPLAY_COLLECTOR_DEPLOYED,
        DISPLAY_DRIVE,
        DISPLAY_AUTO_START
    }

    private enum SystemState {
        DISPLAYING_FAULT,
        DISPLAYING_CLIMB,
        DISPLAYING_BEAK_DOWN,
        DISPLAYING_HAS_CARGO,
        DISPLAYING_COLLECTOR_DEPLOYED,
        DISPLAYING_DRIVE,
        DISPLAYING_AUTO_START
    }

    private void runLoop(double duration) {
        double endTime = Timer.getFPGATimestamp() + duration;
        double timeStamp;

        do {
            timeStamp = Timer.getFPGATimestamp();
            loop.onLoop(timeStamp );
            LEDCanifier.getInstance().readPeriodicInputs();
            LEDCanifier.getInstance().writePeriodicOutputs();
            Timer.delay(Constants.kLooperDt);
        } while (timeStamp < endTime);
    }
}
