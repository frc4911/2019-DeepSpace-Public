package com.team4911.frc2019.subsystems;

import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

    private static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    public enum SystemState {
        IDLE,
        MANUAL,
        COLLECTING_CARGO,
        DEPLOYING_COLLECTOR_EJECTING,
        STOWING_COLLECTOR,
        SHOOTING_CARGO,
        COORDINATING
    }

    public enum WantedState {
        IDLE,
        COLLECT_CARGO,
        STOP_COLLECT_CARGO,
        SHOOT_CARGO,
        STOP_SHOOT_CARGO,
        DEPLOY_COLLECTOR,
        STOW_COLLECTOR,
        WANT_MANUAL,
        COORDINATE
    }

    public enum ElevatorState {
        IDLE,
        HOMING,
        HOLDING,
        MOVING_TO_SETPOINT,
        OPEN_LOOP,
    }

    private SystemState mSystemState = SystemState.IDLE;
    private WantedState mWantedState = WantedState.IDLE;
    private ElevatorState mElevatorState = ElevatorState.IDLE;
    private boolean mStateChanged;

    private Drive mDrive = Drive.getInstance();
    private Collector mCollector = Collector.getInstance();
    private Elevator mElevator = Elevator.getInstance();
    private HatchIntake mHatchIntake = HatchIntake.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private LED mLED = LED.getInstance();

    private boolean mIsGamePieceTrapped = false;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
                mSystemState = SystemState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
                SystemState newState;
                switch (mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case MANUAL:
                        newState = defaultStateTransfer();
                        break;
                    case COLLECTING_CARGO:
                        newState = handleCollectingCargo();
                        break;
                    case DEPLOYING_COLLECTOR_EJECTING:
                        newState = handleDeployCollector(true);
                        break;
                    case STOWING_COLLECTOR:
                        newState = handleStowCollector();
                        break;
                    case SHOOTING_CARGO:
                        newState = defaultStateTransfer();
                        break;
                    case COORDINATING:
                        newState = handleCoordinating();
                        break;
                    default:
                        newState = SystemState.IDLE;
                }

                if (newState != mSystemState) {
                    System.out.println("Superstructure State " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }

                handleLEDState();
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
    };

    private void handleLEDState() {
        boolean blinking = !mDrive.isHighGear();
        if (mShooter.hasCargo()) {
            mLED.setWantedAction(LED.WantedAction.DISPLAY_HAS_CARGO, blinking);
        } else if (mCollector.isDeployed()) {
            mLED.setWantedAction(LED.WantedAction.DISPLAY_COLLECTOR_DEPLOYED, blinking);
        } else if (mHatchIntake.isBeakDown()) {
            mLED.setWantedAction(LED.WantedAction.DISPLAY_BEAK_DOWN, blinking);
        } else {
            mLED.setWantedAction(LED.WantedAction.DISPLAY_DRIVE, blinking);
        }
    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case COLLECT_CARGO:
                return SystemState.COLLECTING_CARGO;
            case DEPLOY_COLLECTOR:
                return SystemState.DEPLOYING_COLLECTOR_EJECTING;
            // case STOP_COLLECT_CARGO:
            case STOW_COLLECTOR:
                return SystemState.STOWING_COLLECTOR;
            case SHOOT_CARGO:
                return SystemState.SHOOTING_CARGO;
            case WANT_MANUAL:
                return SystemState.MANUAL;
            case COORDINATE:
                return SystemState.COORDINATING;
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleIdle() {
        return defaultStateTransfer();
    }

    private SystemState handleCoordinating() {
        if (mSystemState != SystemState.COORDINATING) {
            mWantedState = WantedState.COORDINATE;
        }
        // coordinateElevatorAndCollector();
        return defaultStateTransfer();
    }

    // private SystemState coordinateElevatorAndCollector() {

    // }

    public synchronized void setCoordinationState() {

    }

    // CARGO COLLECTION COORDINATION SEQUENCE
    // Sequence is
    // 1) Pre-conditions: Elevator must be low enough and Shooter doesn't have cargo
    // 2)  State sequence:
    //      a) Deploy Collector & start Shooter intake
    //      b) Loop until has cargo or interrupted
    //          i) Stow collector
    //          ii) Hold Shooter
    //      c) if has cargo
    //          ii) Move Elevator to lowest cargo position

    private enum COLLECTING_CARGO {
        IDLE,
        DEPLOYING,
        COLLECTING_LOW,
        COLLECTING_HIGH,
        WAITING_FOR_ELEVATOR,
        RETRACTING,
    }

    private COLLECTING_CARGO mCollecting_cargo_state = COLLECTING_CARGO.IDLE;

//    private double endTime = Timer.getFPGATimestamp();
    private SystemState handleCollectingCargo() {
        COLLECTING_CARGO newState = mCollecting_cargo_state;

        switch (mCollecting_cargo_state) {
            case DEPLOYING:
                if (mElevator.isSafeToCollect()) {
                    mCollector.setWantedState(Collector.WantedState.DEPLOY);
                    newState = COLLECTING_CARGO.COLLECTING_LOW;
                }
//                endTime = Timer.getFPGATimestamp() + 2.5;
                break;

            case COLLECTING_LOW:
//                if (mShooter.hasCargo() || (endTime < Timer.getFPGATimestamp())) {
                if (mShooter.hasCargo()) {
                    mShooter.setWantedState(Shooter.WantedState.HOLD);
                    mElevator.setPosition(Elevator.PositionPresets.CARGO_LVL_1);
                    newState = COLLECTING_CARGO.WAITING_FOR_ELEVATOR;
                } else if (mElevator.isSafeToRetractCollector()) {
                    mCollector.setWantedState(Collector.WantedState.STOW);
                    newState = COLLECTING_CARGO.COLLECTING_HIGH;
                }
                break;

            case COLLECTING_HIGH:
                if (mShooter.hasCargo()) {
                    mShooter.setWantedState(Shooter.WantedState.HOLD);
                    newState = COLLECTING_CARGO.IDLE;
                } else if (mElevator.isSafeToCollect()) {
                    newState = COLLECTING_CARGO.DEPLOYING;
                } else {
                    newState = COLLECTING_CARGO.COLLECTING_HIGH;
                }
                break;

            case WAITING_FOR_ELEVATOR:
                if (mElevator.isSafeToRetractCollector()) {
                    mCollector.setWantedState(Collector.WantedState.STOW);
                    newState = COLLECTING_CARGO.RETRACTING;
                }
                break;

            case RETRACTING:
                mCollector.setWantedState(Collector.WantedState.STOW);
                newState = COLLECTING_CARGO.IDLE;
                break;

            case IDLE:
            default:
                break;
        }

        if (newState != mCollecting_cargo_state) {
            System.out.println("Superstucture/Collecting Cargo state " + mCollecting_cargo_state + " to " + newState);
            mCollecting_cargo_state = newState;
        }

        return collectingCargoStateTransfer();
    }

    private SystemState collectingCargoStateTransfer() {
        switch (mWantedState) {
            case COLLECT_CARGO:
                return SystemState.COLLECTING_CARGO;
            case DEPLOY_COLLECTOR:
                return SystemState.DEPLOYING_COLLECTOR_EJECTING;
            case STOW_COLLECTOR:
                return SystemState.STOWING_COLLECTOR;
            case SHOOT_CARGO:
                if (mElevator.isSafeToRetractCollector()) {
                    mCollector.setWantedState(Collector.WantedState.STOW);
                }
                return SystemState.SHOOTING_CARGO;
            case STOP_COLLECT_CARGO:
            default:
                if (!mShooter.hasCargo() || mElevator.isSafeToRetractCollector()) {
                    mShooter.setWantedState(Shooter.WantedState.HOLD);
                    mCollector.setWantedState(Collector.WantedState.STOW);
                }

                return SystemState.IDLE;
        }
    }

    private SystemState handleDeployCollector(boolean isEjecting) {
        if (isEjecting) {
            mCollector.setWantedState(Collector.WantedState.DEPLOY_AND_EJECT);
        } else {
            mCollector.setWantedState(Collector.WantedState.DEPLOY);
        }

        return SystemState.IDLE;
    }

    private SystemState handleStowCollector() {
        if (!mShooter.hasCargo() || mElevator.isSafeToRetractCollector()) {
            mCollector.setWantedState(Collector.WantedState.STOW);
            // mShooter.setWantedState(Shooter.WantedState.HOLD);
            return SystemState.IDLE;
        }

        return SystemState.STOWING_COLLECTOR;
    }

    private SystemState handleCollectingHatch() {
        mElevator.setPosition(Elevator.PositionPresets.CARGO_LVL_1); // TODO: need to change this
        mHatchIntake.setWantedState(HatchIntake.WantedState.STOW);

        return collectingHatchStateTransfer();
    }

    private SystemState collectingHatchStateTransfer() {
        // switch(mWantedState) {
        //     case STOW_HATCH:
        //         return SystemState.STOWING_HATCH;
        //     case COLLECT_CARGO:
        //         mHatchIntake.setWantedState(HatchIntake.WantedState.STOW);
        //         mElevator.setPosition(Elevator.PositionPresets.HATCH_LVL_1);
        //         return SystemState.COLLECTING_CARGO;
        //     case STOW_CARGO:
        //         mHatchIntake.setWantedState(HatchIntake.WantedState.STOW);
        //         return SystemState.STOWING_CARGO;
        //     default:
        return SystemState.IDLE;
        // }
    }

    public synchronized boolean isGamePieceTrapped() {
        return mIsGamePieceTrapped;
    }

    public synchronized void setGamePieceTrapped(boolean isGamePieceTrapped) {
        SmartDashboard.putBoolean("Superstructure/isGamePieceTrapped", isGamePieceTrapped);
        mIsGamePieceTrapped = isGamePieceTrapped;
    }

    public synchronized void setWantedState(WantedState state) {
        WantedState newState = mWantedState;

        switch (state) {
            case COLLECT_CARGO:
//                System.out.println("In COLLECT_CARGO:" +
//                        " mWantedState=" + mWantedState +
//                        " isGamePieceTrapped=" + isGamePieceTrapped() +
//                        " mElevator.isSafeToCollect()=" + mElevator.isSafeToCollect() +
//                        " mShooter.hasCargo()= " + mShooter.hasCargo());
                if (state != mWantedState) {
                    // Also check whether to any unwind / stop other states
                    if (isGamePieceTrapped()) {
                        newState = WantedState.DEPLOY_COLLECTOR;
                    } else if (!mShooter.hasCargo()) {
                        mShooter.setWantedState(Shooter.WantedState.INTAKE);
                        newState = state;
                        if (mElevator.isSafeToCollect()) {
//                        System.out.println("In COLLECT_CARGO and Elevator safe so deploy collector");
                            mCollecting_cargo_state = COLLECTING_CARGO.DEPLOYING;
                        } else {
                            mCollecting_cargo_state = COLLECTING_CARGO.COLLECTING_HIGH;
//                        System.out.println("In COLLECT_CARGO but Elevator not safe so just spin shooter");
                        }
                    } else {
//                        System.out.println("In COLLECT_CARGO but Elevator not safe or has cargo");
                    }
                }
                break;

            case STOP_COLLECT_CARGO:
                // if (state != mWantedState) {
                //     if (isGamePieceTrapped()) {
                //         newState = WantedState.STOW_COLLECTOR;
                //     } else {
                        newState = state;
                //     }
                // }

                break;

            // CARGO SHOOTING COORDINATION
            //
            // 1) Pre-conditions:  Shooter must have cargo.
            // 2) If running Cargo Collection sequence, interrupt it.
            //
            case SHOOT_CARGO:
                newState = mWantedState;
                if (mWantedState != WantedState.COLLECT_CARGO) {
                    if (mShooter.hasCargo()) {
                        mShooter.setWantedState(Shooter.WantedState.SHOOT);
                        newState = state;
                    }
                }

                break;

            case STOP_SHOOT_CARGO:
                newState = mWantedState;
                if (mWantedState != WantedState.COLLECT_CARGO) {
                    mShooter.setWantedState(Shooter.WantedState.HOLD);
                    newState = WantedState.IDLE;
                }
                break;

            case IDLE:
                newState = WantedState.IDLE;
                break;

              default:
                newState = state;
        }

        mWantedState = newState;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }
}