package com.team4911.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.subsystems.Subsystem;
import com.team254.lib.util.RobotName;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.Robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
The Elevator subsystem has two motors, an encoder, a string pot, and two limit switches
A positive input to the motor moves it up
A negative input to the motor moves it down
The encoder is used for the motion magic/position pid
The string pot is used to help the encoder know its position during start up
The limit switches are configured t
*/

public class Elevator extends Subsystem {

    private static Elevator sInstance = null;
    public static ElevatorTest mElevatorTest = new ElevatorTest();
    public Robot mRobot = null;

    public static Elevator getInstance() {
        if(sInstance == null) {
            sInstance = new Elevator();
        }
        return sInstance;
    }

    // Hardware
    private final TalonSRX mTalonMaster, mTalonSlave;
    private final AnalogInput mStringPot;

    private final int kPositionControlSlot = 0;
    private final int kMotionMagicControlSlot = 1;
    private final double kHoldRange;
    private final int kInitEncRange = 100;

    // collect these values from a single point near bottom of travel
    private final double kPotAtBottom; // pot reading at bottom
    private final double kEncAtBottom; // encoder reading at bottom
    private final double kInchesAtBottom; // measured from floor to middle of shooter axle

    // collect these values from a single point near top of travel
    private final double kPotAtTop;
    private final double kEncAtTop;
    private final double kInchesAtTop;

    private final double kUpperSoftLimit;
    private final double kLowerSoftLimit;

    // kEncTicksPerPot is the slope of the encoder/pot
    private final double kEncTicksPerPot;
    // y = mx + b  m is the slope, y is encoder ticks, x is pot value, b is the y intercept, 
    // this calculates b from the bottom values
    private final double kEncYIntercept;

    private final double kInchesYIntercept;
    // encoder ticks per inch
    private final double kTicksPerInch;
    // joystick movement threshold
    private final double kJoystickThreshold = 0.15;


    private double kMinSpeed = 0.1;
    private double kMaxSpeed = 1.0;
    private double slopeLength = 6.0; // increase this value to lengthen ramp down as elevator approaches bottom
    private double ramp_slope;
    private double ramp_b;

    // presets - measured in inches from ground to center of shooter axle
    private final double kPresetHatchLvl1 = 19.0;
    private final double kPresetCargoLvl1 = 33.5;
    private final double kPresetHatchLvl2 = 47.0;
    private final double kPresetCargoShip = 48.0;
    private final double kPresetCargoLvl2 = 61.5;
    private final double kPresetHatchLvl3 = 75.0;
    private final double kPresetCargoLvl3 = 89.5;

    // set limits .5 inches beyond most extreme targets
    private final double kLowestTravelLimit = kPresetHatchLvl1;
    private final double kHighestTravelLimit = kPresetCargoLvl3 + 0.5;
    
    // must be below this to intake cargo, 3.0 is inches
    private double kSafeIntakeThreshold = 0.0;
    private double kSafeIntakeToRetractWhenHaveCargo;
    public enum SystemState {
        IDLE,
        HOMING,
        HOLDING,
        MOVING_TO_SETPOINT,
        OPEN_LOOP,
        BRINGING_UP,
        SELF_TESTING
    }

    public enum WantedState {
        IDLE,
        HOME,
        HOLD,
        MOVE_TO_SETPOINT,
        OPEN_LOOP,
        BRING_UP,
        SELF_TEST
    }

    public enum PositionPresets {
        CARGO_LVL_1,
        CARGO_LVL_2,
        CARGO_LVL_3,
        CARGO_SHIP,
        HATCH_LVL_1,
        HATCH_LVL_2,
        HATCH_LVL_3
    }

    protected SystemState mSystemState = SystemState.IDLE;
    protected WantedState mWantedState = WantedState.IDLE;
    private PositionPresets mPositionPresets = PositionPresets.HATCH_LVL_1;
    protected PeriodicIO mPeriodicIO;
    protected boolean mStateChanged;
    private boolean mManualOverride = false;
    private SlotConfiguration mSlotConfigurationPositionControl;
    private SlotConfiguration mSlotConfigurationMotionMagicControl;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private boolean mLoggingEnabled = false;

    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized(Elevator.this) {
                mSystemState = SystemState.IDLE;
                mWantedState = WantedState.IDLE;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized(Elevator.this) {
                SystemState newState;
                switch(mSystemState) {
                    case IDLE:
                        newState = handleIdle();
                        break;
                    case HOMING:
                        newState = handleHoming();
                        break;
                    case HOLDING:
                        newState = handleHolding();
                        break;
                    case MOVING_TO_SETPOINT:
                        newState = handleMovingToSetpoint(timestamp);
                        break;
                    case OPEN_LOOP:
                        newState = handleOpenLoop();
                        break;
                    case BRINGING_UP:
                        newState = mElevatorTest.handleBringingUp();
                        break;
                    case SELF_TESTING:
                        newState = mElevatorTest.handleSelfTesting(timestamp);
                        break;
                    default:
                        newState = SystemState.HOLDING;
                }

                if (newState != mSystemState) {
                    System.out.println("Elevator state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            mTalonMaster.set(ControlMode.PercentOutput, 0.0);
        }
    };

    public Elevator() {

        if (RobotName.name.equals(Constants.kSilverName)){
            // collect these values from a single point near bottom of travel
            kPotAtBottom = 138.0; // pot reading near bottom
            kEncAtBottom = 8644.0; // encoder reading near bottom
            kInchesAtBottom = 20.0; // measured from floor to middle of shooter axle

            // collect these values from a single point near top of travel
            kPotAtTop = 3203.0;
            kEncAtTop = 126515;
            kInchesAtTop = 84;
        }
        else {
            // for Chrome

            // collect these values from a single point near bottom of travel
            kPotAtBottom = 87.0; // pot reading at bottom
            kEncAtBottom = 6688.0; // encoder reading at bottom
            kInchesAtBottom = 19.875; // measured from floor to middle of shooter axle

            // collect these values from a single point near top of travel
            kPotAtTop = 3049.0;
            kEncAtTop = 120111;
            kInchesAtTop = 82.25;
        }

        // kEncTicksPerPot is the slope of the encoder/pot
        kEncTicksPerPot = (kEncAtTop-kEncAtBottom)/(kPotAtTop-kPotAtBottom);
        // y = mx + b  m is the slope, y is encoder ticks, x is pot value, b is the y intercept, 
        // this calculates b from the bottom values
        kEncYIntercept = kEncAtBottom-kPotAtBottom*kEncTicksPerPot;
        // encoder ticks per inch
        kTicksPerInch = (kEncAtTop-kEncAtBottom)/(kInchesAtTop-kInchesAtBottom);

        kInchesYIntercept = kEncAtBottom - kInchesAtBottom * kTicksPerInch;

        kHoldRange = kTicksPerInch/4.0;

        // calculate soft limits in inches from the ground
        kUpperSoftLimit = convertInchesToTicks(kHighestTravelLimit);
        kLowerSoftLimit = convertInchesToTicks(kLowestTravelLimit);

        // calculate slope and b for line equation of ramp down as elevator approches bottom
        ramp_slope = (kMaxSpeed-kMinSpeed)/(convertInchesToTicks(slopeLength));
        ramp_b = kMinSpeed-ramp_slope*kPresetHatchLvl1;
    
        System.out.println("Elevator kEncTicksPerPot = "+kEncTicksPerPot);
        System.out.println("Elevator kTicksPerInch = "+kTicksPerInch);
        System.out.println("Elevator kEncYIntercept = "+kEncYIntercept);
        System.out.println("Elevator kInchesYIntercept = "+kInchesYIntercept);
        System.out.println("Elevator kUpperSoftLimit = "+kUpperSoftLimit);
        System.out.println("Elevator kLowerSoftLimit = "+kLowerSoftLimit);

        mPeriodicIO = new PeriodicIO();

        mTalonMaster = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMasterId);
        mTalonMaster.setInverted(true);

        mTalonSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorSlaveId, Constants.kElevatorMasterId);
        mTalonSlave.setInverted(false);

        mTalonMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kLongCANTimeoutMs);
        mTalonMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.kMagEncFramePer, Constants.kLongCANTimeoutMs);
        
        mStringPot = new AnalogInput(Constants.kElevatorPotId);

        kSafeIntakeThreshold = convertInchesToTicks(kPresetHatchLvl1 + 3.0);
        kSafeIntakeToRetractWhenHaveCargo = convertInchesToTicks(kPresetHatchLvl1 + 7.0); // + 4.0

        // Apply voltage compensation
        // TalonSRXUtil.checkError(
        //         mTalonMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs),
        //         "Could not set voltage compensation saturation elevator: ");

        // Limit current draw
        //TODO: consider setting peak to 0 and continuous to 30 or something
        TalonSRXUtil.checkError(
            mTalonMaster.configContinuousCurrentLimit(20, Constants.kLongCANTimeoutMs),
            "Could not set elevator continuous current limit.");

        TalonSRXUtil.checkError(
            mTalonMaster.configPeakCurrentLimit(35, Constants.kLongCANTimeoutMs),
            "Could not set elevator peak current limit.");

        TalonSRXUtil.checkError(
            mTalonMaster.configPeakCurrentDuration(200, Constants.kLongCANTimeoutMs),
            "Could not set elevator peak current duration.");

        mTalonMaster.enableCurrentLimit(true);

        mSlotConfigurationPositionControl = new SlotConfiguration();
        mSlotConfigurationMotionMagicControl = new SlotConfiguration();

        configLimitSwitches();
        configSoftLimits();
        loadPositionGains();
        loadMotionMagicGains();
        setNeutralMode(NeutralMode.Brake);

        mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/ELEVATOR-LOGS.csv", Elevator.PeriodicIO.class);
    }

    private void loadPositionGains() {
        mSlotConfigurationPositionControl.kP = Constants.kElevatorPositionKp;
        mSlotConfigurationPositionControl.kI = Constants.kElevatorPositionKi;
        mSlotConfigurationPositionControl.kD = Constants.kElevatorPositionKd;
        mSlotConfigurationPositionControl.kF = Constants.kElevatorPositionKf;
        mSlotConfigurationPositionControl.integralZone = Constants.kElevatorPositionIZone;
        mTalonMaster.configureSlot(mSlotConfigurationPositionControl, kPositionControlSlot, Constants.kLongCANTimeoutMs);
    }

    private void loadMotionMagicGains() {
        mSlotConfigurationMotionMagicControl.kP = Constants.kElevatorMotionMagicKp;
        mSlotConfigurationMotionMagicControl.kI = Constants.kElevatorMotionMagicKi;
        mSlotConfigurationMotionMagicControl.kD = Constants.kElevatorMotionMagicKd;
        mSlotConfigurationMotionMagicControl.kF = Constants.kElevatorMotionMagicKf;
        mSlotConfigurationMotionMagicControl.integralZone = Constants.kElevatorMotionMagicIZone;
        mTalonMaster.configMotionCruiseVelocity(Constants.kElevatorCruiseVelocity, Constants.kLongCANTimeoutMs);
        mTalonMaster.configMotionAcceleration(Constants.kElevatorAccelerationUp, Constants.kLongCANTimeoutMs);
        mTalonMaster.configureSlot(mSlotConfigurationMotionMagicControl, kMotionMagicControlSlot, Constants.kLongCANTimeoutMs);
    }

    private void configMotionMagic(boolean upwards) {
        if (upwards) {
            mTalonMaster.configMotionAcceleration(Constants.kElevatorAccelerationUp, Constants.kLongCANTimeoutMs);
            mTalonMaster.configOpenloopRamp(Constants.kElevatorVoltageRampRate, Constants.kLongCANTimeoutMs);
            mTalonMaster.configClosedloopRamp(Constants.kElevatorVoltageRampRate, Constants.kLongCANTimeoutMs);
        } else {
            mTalonMaster.configMotionAcceleration(Constants.kElevatorAccelerationDown, Constants.kLongCANTimeoutMs);
            mTalonMaster.configOpenloopRamp(0.0, Constants.kLongCANTimeoutMs);
            mTalonMaster.configClosedloopRamp(0.0, Constants.kLongCANTimeoutMs);
        }
    }

    private void configLimitSwitches() {
        mTalonMaster.overrideLimitSwitchesEnable(true);
        mTalonMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs);
        mTalonMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, Constants.kLongCANTimeoutMs);
        // mTalonMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, Constants.kLongCANTimeoutMs);
    }
    
    private void configSoftLimits() {
        mTalonMaster.overrideSoftLimitsEnable(true);
        mTalonMaster.configForwardSoftLimitThreshold((int)kUpperSoftLimit, Constants.kLongCANTimeoutMs);
        mTalonMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        mTalonMaster.configReverseSoftLimitThreshold((int)kLowerSoftLimit, Constants.kLongCANTimeoutMs);
        mTalonMaster.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
    }

    private void setNeutralMode(NeutralMode mode) {
        mTalonMaster.setNeutralMode(mode);
        mTalonSlave.setNeutralMode(mode);
    }

    public void registerRobot(Robot robot){
        mRobot = robot;
    }

    protected SystemState defaultStateTransfer() {
        switch(mWantedState) {
            case HOME:
                return SystemState.HOMING;
            case HOLD:
                return SystemState.HOLDING;
            case MOVE_TO_SETPOINT:
                return SystemState.MOVING_TO_SETPOINT;
            case OPEN_LOOP:
                return SystemState.OPEN_LOOP;
            case BRING_UP:
                return SystemState.BRINGING_UP;
            case SELF_TEST:
                return SystemState.SELF_TESTING;
            default:
                return SystemState.HOMING;
        }
    }

    private SystemState handleIdle() {
        mPeriodicIO.demand = 0.0;
        mPeriodicIO.feedforward = 0.0;
        mWantedState = WantedState.HOME;
        return defaultStateTransfer();
    }

    private SystemState handleHoming() {
        int position = (int) convertPotValueToTicks(mPeriodicIO.string_pot_value);

        if((Math.abs(mPeriodicIO.position_ticks - position) < kInitEncRange) && !mStateChanged) {
            if ((mRobot != null) && (mRobot.isTest())){
                mWantedState = WantedState.BRING_UP;
            }
            else{
                mWantedState = WantedState.HOLD;
            }
        } else {
            mTalonMaster.setSelectedSensorPosition(position, 0, 10);
        }
        return defaultStateTransfer();
    }

    private SystemState handleHolding() {
        if(mStateChanged && !isManualOverride()) {
            mTalonMaster.selectProfileSlot(kPositionControlSlot, 0);
            mPeriodicIO.position_demand = mPeriodicIO.position_ticks; // TODO: look into this more to reduce jerk when started
        }
        return defaultStateTransfer();
    }

    boolean mApproachFromBelow = true;
    double mLastError = Double.MAX_VALUE;
    double mMMTimeout = 0;

    private SystemState handleMovingToSetpoint(double now) {
        if(!isManualOverride()) {
            
            double preset = 0.0;
            switch(mPositionPresets) {
                case CARGO_LVL_1:
                    preset = convertInchesToTicks(kPresetCargoLvl1); //TODO: do conversion once at startup
                    // System.out.println(mPositionPresets.toString()+" inches "+kPresetCargoLvl1+" enc="+preset);
                    break;
                case CARGO_LVL_2:
                    preset = convertInchesToTicks(kPresetCargoLvl2);
                    // System.out.println(mPositionPresets.toString()+" inches "+kPresetCargoLvl2+" enc="+preset);
                    break;
                case CARGO_LVL_3:
                    preset = convertInchesToTicks(kPresetCargoLvl3);
                    // System.out.println(mPositionPresets.toString()+" inches "+kPresetCargoLvl3+" enc="+preset);
                    break;
                case CARGO_SHIP:
                    preset = convertInchesToTicks(kPresetCargoShip);
                    // System.out.println(mPositionPresets.toString()+" inches "+kPresetCargoShip+" enc="+preset);
                    break;
                case HATCH_LVL_1:
                    preset = convertInchesToTicks(kPresetHatchLvl1);
                    // System.out.println(mPositionPresets.toString()+" inches "+kPresetHatchLvl1+" enc="+preset);
                    break;
                case HATCH_LVL_2:
                    preset = convertInchesToTicks(kPresetHatchLvl2);
                    // System.out.println(mPositionPresets.toString()+" inches "+kPresetHatchLvl2+" enc="+preset);
                    break;
                case HATCH_LVL_3:
                    preset = convertInchesToTicks(kPresetHatchLvl3);
                    // System.out.println(mPositionPresets.toString()+" inches "+kPresetHatchLvl3+" enc="+preset);
                    break;
                default:
                    preset = convertInchesToTicks(kPresetHatchLvl1);
            }

            if (mStateChanged || (mPeriodicIO.position_demand != preset)){
                mApproachFromBelow = mPeriodicIO.position_ticks < preset?true:false;
                mLastError = Math.abs(preset - mPeriodicIO.position_ticks);
                mMMTimeout = now + 0.75; // .75 seconds to get going
                if (mApproachFromBelow) {
                    configMotionMagic(true);
                } else {
                    configMotionMagic(false);
                }
            }
            mPeriodicIO.position_demand = preset;

            // error is positive if current pos is below target
            double error = mPeriodicIO.position_demand - mPeriodicIO.position_ticks;

            if ((mApproachFromBelow && error<0) || (!mApproachFromBelow && error>0)){
                mWantedState = WantedState.HOLD;
            }
            else {
                error = Math.abs(error);
                if (mLastError > error){
                    mLastError = error;
                    mMMTimeout = now + .25; // if it has not improved in .25 seconds then we are done
                }
                else if ((now > mMMTimeout) || (error < kHoldRange)){
                    mWantedState = WantedState.HOLD;
                }
            }
        }
        return defaultStateTransfer();
    }

    private SystemState handleOpenLoop() {
        if(Math.abs(mPeriodicIO.demand) < kJoystickThreshold) {
            mWantedState = WantedState.HOLD;
        }
        return defaultStateTransfer();
    }

    public synchronized void setWantedState(WantedState state) {
        mWantedState = state;
    }

    public synchronized WantedState getWantedState() {
        return mWantedState;
    }

    public synchronized void setOpenLoop(double speed) {
        if(Math.abs(speed) > kJoystickThreshold) {
            if(mSystemState != SystemState.OPEN_LOOP) {
                mWantedState = WantedState.OPEN_LOOP;
            }
            mPeriodicIO.demand = speed;
        } else {
            mPeriodicIO.demand = 0.0;
        }
        mPeriodicIO.feedforward = 0.0;
    }

    public synchronized void setPosition(PositionPresets position) {
        if(mSystemState != SystemState.MOVING_TO_SETPOINT) {
            mWantedState = WantedState.MOVE_TO_SETPOINT;
        }
        mPositionPresets = position;
    }

    public synchronized void setManualOverride(boolean manualOverride) {
        mManualOverride = manualOverride; // TODO: write code to reset
    }

    public synchronized boolean isManualOverride() {
        return mManualOverride;
    }

    // returns true if elevator is close enough to the bottom to intake cargo
    public synchronized boolean isSafeToCollect() {
        return mPeriodicIO.position_ticks < kSafeIntakeThreshold;
    }

    /**
     * Returns whether elevator position above safe limit to retract collector.
     *
     * @return <code>true</code> when elevator position is above; otherwise <code>false</code>
     * */
    public synchronized boolean isSafeToRetractCollector() {
        return kSafeIntakeToRetractWhenHaveCargo <= mPeriodicIO.position_ticks;
    }

    // inches - inches is from ground to center of shooter axle
    // return - encoder value for that position
    private double convertInchesToTicks(double inches) {
        return (inches * kTicksPerInch) + kInchesYIntercept;
    }

    // returns error (in encoder ticks) between expected and actual encoder readings
    private void checkPotAndEncoder(){
        mPeriodicIO.test_error = convertPotValueToTicks(mPeriodicIO.string_pot_value) - mPeriodicIO.position_ticks;
    }

    // given a pot value it returns expected ticks
    protected double convertPotValueToTicks(int pot) {
        return (pot * kEncTicksPerPot) + kEncYIntercept;
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
        mPeriodicIO.string_pot_value = mStringPot.getValue();
        mPeriodicIO.position_ticks = mTalonMaster.getSelectedSensorPosition(0);
        // mPeriodicIO.top_limit = !mTalonMaster.getSensorCollection().isFwdLimitSwitchClosed();
        mPeriodicIO.bottom_limit = !mTalonMaster.getSensorCollection().isRevLimitSwitchClosed();
        mPeriodicIO.master_current = mTalonMaster.getOutputCurrent();
        mPeriodicIO.slave_current = mTalonSlave.getOutputCurrent();
        mPeriodicIO.distance = mPeriodicIO.position_ticks / kTicksPerInch;
        checkPotAndEncoder();

        if (mLoggingEnabled && mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }
    
    int printCnt = 0;
    @Override
    public void writePeriodicOutputs() {
            
        // TODO: remove feedforward if appropriate
        if(mSystemState == SystemState.HOLDING) {
            // this addresses a problem with the encoder on Chrome. 
            // It seems to slip over time causing the presets to be low
            // Only do this when at the bottom and the encoder is off by more than 1 inch
            if (mPeriodicIO.bottom_limit && mPeriodicIO.string_pot_value < 600){
                // suspend PID when resetting encoder
                mTalonMaster.set(ControlMode.PercentOutput, 0);
                // calculate proper setting for current location
                int position = (int) convertPotValueToTicks(mPeriodicIO.string_pot_value);
                // reset the position
                mTalonMaster.setSelectedSensorPosition(position, 0, 10);

                // System.out.println("reset elevator enc " + mPeriodicIO.test_error + " " + mPeriodicIO.position_ticks);
            }
            else{
                mTalonMaster.set(ControlMode.Position, mPeriodicIO.position_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
            }
        } else if(mSystemState == SystemState.MOVING_TO_SETPOINT) {
            if(!mStateChanged) {
                mTalonMaster.set(ControlMode.MotionMagic, mPeriodicIO.position_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
            }
        } else {
            double speed = Math.pow(mPeriodicIO.demand,3);// Math.pow(mPeriodicIO.demand,2)*Math.signum(mPeriodicIO.demand);
            // uncomment to implement slow down at bottom of travel
            // if (speed < 0){
            //     double rampSpeed = ramp_slope*mPeriodicIO.position_ticks+ramp_b;
            //     speed = Math.min(rampSpeed, speed);
            // }
            mTalonMaster.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {

        // TODO: consider reducing list for competition
        SmartDashboard.putString("Elevator/SystemState", mSystemState.toString());
        // SmartDashboard.putString("Elevator/WantedState", mWantedState.toString());
        SmartDashboard.putString("Elevator/Position", mPositionPresets.toString());
        // SmartDashboard.putNumber("Elevator/Percent Output", mPeriodicIO.demand);
        SmartDashboard.putNumber("Elevator/Setpoint", mPeriodicIO.position_demand);
        SmartDashboard.putNumber("Elevator/Ticks", mPeriodicIO.position_ticks);
        SmartDashboard.putNumber("Elevator/Pot", mPeriodicIO.string_pot_value);
        SmartDashboard.putNumber("Elevator/Master Current", mPeriodicIO.master_current);
        SmartDashboard.putNumber("Elevator/Slave Current", mPeriodicIO.slave_current);
        // SmartDashboard.putNumber("Elevator/Distance", mPeriodicIO.distance);
        SmartDashboard.putNumber("Elevator/Encoder-Pot Error", mPeriodicIO.test_error);
        SmartDashboard.putBoolean("Elevator/Bottom Limit", mPeriodicIO.bottom_limit);
    }

    @Override
    public void stop() {
        mTalonMaster.set(ControlMode.PercentOutput, 0.0);
        stopLogging();
    }

    @Override
    public void zeroSensors() {
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    public static class PeriodicIO {
        // INPUTS
        public int string_pot_value;
        public int position_ticks;
        // public boolean top_limit;
        public boolean bottom_limit;
        public double distance;
        public double master_current;
        public double slave_current;
        public double test_error;

        // OUTPUTS
        public double demand;
        public double position_demand;
        public double feedforward;
    }
}