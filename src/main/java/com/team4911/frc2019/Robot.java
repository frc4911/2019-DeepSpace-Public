package com.team4911.frc2019;

import com.team254.lib.autos.AutoModeBase;
import com.team254.lib.autos.AutoModeExecutor;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.loops.Looper;
import com.team254.lib.subsystems.SubsystemManager;
import com.team254.lib.util.CheesyDriveHelper;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.LatchedBooleanFalse;
import com.team254.lib.util.RobotName;
import com.team4911.frc2019.auto.AutoModeSelector;
import com.team4911.frc2019.controlboard.ControlBoard;
import com.team4911.frc2019.subsystems.*;
import com.team4911.frc2019.utils.*;
import com.team4911.frc2019.paths.TrajectoryGenerator;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends TimedRobot {
    private RobotName rrn = new RobotName (Constants.kChromeName);
    private Looper mEnabledLooper = new Looper(Constants.kLooperDt);
    private Looper mDisabledLooper = new Looper(Constants.kLooperDt);
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private ControlBoard mControlBoard = ControlBoard.getInstance();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
        Arrays.asList(
                RobotStateEstimator.getInstance(),
                Drive.getInstance(),
                Superstructure.getInstance(),
                Shooter.getInstance(),
                HatchIntake.getInstance(),
                Collector.getInstance(),
                Elevator.getInstance(),
                LEDCanifier.getInstance()
                )
    );

    private Drive mDrive = Drive.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private HatchIntake mHatchIntake = HatchIntake.getInstance();
    private Elevator mElevator = Elevator.getInstance();
    private LED mLED = LED.getInstance();
    private Collector mCollector = Collector.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private LogWriter mLogWriter = new LogWriter();
    private TelemetryWriter mTelemetryWriter = new TelemetryWriter();
    private DoubleSolenoid mClimberDeploy = new DoubleSolenoid(Constants.kClimberDeployId, Constants.kClimberRetractId);
    private boolean mClimberIsDeployed = false;

    private AutoModeExecutor mAutoModeExecutor;

    private int driveChoice = 0;
    TimeCheck watch = new TimeCheck(.021,"long periodic");
    CameraManipulator cameraManipulator = new CameraManipulator();
    UsbCamera camera0 = null;
    UsbCamera camera1 = null;
    InfoDumper infoDumper = new InfoDumper();
    LatchedBoolean lbDrive = new LatchedBoolean();
    LatchedBoolean lbElevator = new LatchedBoolean();
    LatchedBoolean lbHatch = new LatchedBoolean();

    boolean mAbortAuto = false;

    // TODO: Consider moving into it's own class
    private enum TrappedGamePiece {
        NONE,
        CARGO,
        HATCH
    }

    private SendableChooser<TrappedGamePiece> mTrappedGamePieceChooser;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();
            // camera0 = cameraManipulator.startCamera("camera0", "/dev/video0");
            // camera1 = cameraManipulator.startCamera("camera1", "/dev/video1");

            // Turn off Limelight LED and set to Driver mode
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);

            // each subsystem uses this to know when it is in test mode
            mCollector.registerRobot(this);
            mDrive.registerRobot(this);
            mElevator.registerRobot(this);
            mHatchIntake.registerRobot(this);
            mShooter.registerRobot(this);

            // Only log every other Looper cycle
            mLogWriter.registerSubsystemManager(mSubsystemManager, (long)(1 / Constants.kLooperDt) * 2, Thread.NORM_PRIORITY - 1);

            mTelemetryWriter.registerSubsystemManager(mSubsystemManager, (long)(1 / Constants.kLooperDt) * 2, Thread.NORM_PRIORITY - 1);

            mTelemetryWriter.start();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mLED.registerEnabledLoops(mEnabledLooper);
            mLED.registerEnabledLoops(mDisabledLooper);

            mTrajectoryGenerator.generateTrajectories();
            mAutoModeSelector.updateModeCreator();

            mTrappedGamePieceChooser = new SendableChooser<TrappedGamePiece>();
            mTrappedGamePieceChooser.setDefaultOption("None", TrappedGamePiece.NONE);
            mTrappedGamePieceChooser.addOption("Cargo", TrappedGamePiece.CARGO);
            mTrappedGamePieceChooser.addOption("Hatch", TrappedGamePiece.HATCH);
            SmartDashboard.putData("Trapped Game Piece", mTrappedGamePieceChooser);

            mClimberDeploy.set(Value.kReverse);
            mClimberIsDeployed = false;

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            // Reset all auto mode state.
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();

            mLED.setEnableFaults(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();

            mLED.setWantedAction(LED.WantedAction.DISPLAY_AUTO_START, false);

            mDisabledLooper.stop();
            Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());        
            mAutoModeExecutor.start();
            mLED.setEnableFaults(false);
            mEnabledLooper.start(); 
            mAbortAuto = false;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();
            mLED.setEnableFaults(false);

            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));


        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            mDisabledLooper.stop();
            mEnabledLooper.start();

            startLogging();
            mLogWriter.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            watch.checkTime();
            if (mControlBoard.getInfoDump()){
                infoDumper.dumpInfo();
            }
            outputToSmartDashboard();
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        outputToSmartDashboard();
        try {
            // TODO:  Add auto or teleop code
            if (Math.abs(mControlBoard.getThrottleRight()) > 0.15 && !mAbortAuto) {
                mAutoModeExecutor.stop();
                mAbortAuto = true;
            }

            if (mAbortAuto) {
                teleopRoutines();
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            watch.checkTime();
            teleopRoutines();
           } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
        testRoutines();
    }

    public void teleopRoutines() {
        mSuperstructure.setGamePieceTrapped(mTrappedGamePieceChooser.getSelected() != TrappedGamePiece.NONE);
        driveRoutine();
        shooterRoutine();
        hatchIntakeRoutine();
        elevatorRoutine();
        collectorRoutine();
        climberRoutine();
        outputToSmartDashboard();
    }

    public void driveRoutine() {
        DriveSignal driveSignal = null;
        String driveChoiceName = null;
        double throttleRight = mControlBoard.getThrottleRight();
        double throttleLeft = mControlBoard.getThrottleLeft();
        double turn = mControlBoard.getTurn();

        if (mControlBoard.getHighGear()) {
            mDrive.setHighGear(true);
        } else if (mControlBoard.getLowGear()) {
            mDrive.setHighGear(false);
        }

        if (mControlBoard.getDriveChoice()) {
            driveChoice++;
        }

        switch (driveChoice % 3) {
            case 0:
                driveSignal = mCheesyDriveHelper.arcadeDrive(throttleRight, turn, false);
                driveChoiceName = "Arcade Drive";
                break;
            case 1:
                driveSignal = mCheesyDriveHelper.cheesyDrive(throttleRight, turn, mControlBoard.getLowGear(), false);
                driveChoiceName = "Cheesy Drive";
                break;
            default:
                driveSignal = mCheesyDriveHelper.tankDrive(throttleLeft, throttleRight, false);
                driveChoiceName = "Tank Drive";
                break;
        }

        SmartDashboard.putString("Drive Mode", driveChoiceName);
        mDrive.setOpenLoop(driveSignal);
    }

    LatchedBoolean shootChargoPressed = new LatchedBoolean();
    LatchedBooleanFalse shootChargoReleased = new LatchedBooleanFalse();

    public void shooterRoutine() {
        boolean button = mControlBoard.getShootCargo();

        if (shootChargoPressed.update(button)) {
            mShooter.setWantedState(Shooter.WantedState.SHOOT);
        }

        if (shootChargoReleased .update(button)) {
            mShooter.setWantedState(Shooter.WantedState.HOLD);
        }
    }

    public void hatchIntakeRoutine() {
        if (mControlBoard.getCollectHatch()) {
            mHatchIntake.setWantedState(HatchIntake.WantedState.OPEN);
        } else if (mControlBoard.getStowHatch()) {
            mHatchIntake.setWantedState(HatchIntake.WantedState.STOW);
        } else if (mControlBoard.getEjectHatch()) {
            mHatchIntake.setWantedState(HatchIntake.WantedState.OPEN_AND_EJECT);
        } else if(mControlBoard.getRetractHatch()) {
            mHatchIntake.setWantedState(HatchIntake.WantedState.STOW);
        }
    }

    LatchedBoolean collectCargoPressed = new LatchedBoolean();
    LatchedBooleanFalse collectCargoReleased = new LatchedBooleanFalse();

    public void collectorRoutine() {
        boolean button = mControlBoard.getCollectCargo();

        if (collectCargoPressed.update(button)) {
            mSuperstructure.setWantedState(Superstructure.WantedState.COLLECT_CARGO);
        }

        if (collectCargoReleased.update(button)) {
            mSuperstructure.setWantedState(Superstructure.WantedState.STOP_COLLECT_CARGO);
        }
    }

    public void elevatorRoutine() {
        mElevator.setOpenLoop(mControlBoard.getMoveElevator());
        if(mControlBoard.getCargo1Preset()) { 
            mElevator.setPosition(Elevator.PositionPresets.CARGO_LVL_1); 
        } else if(mControlBoard.getCargo2Preset()) { 
            mElevator.setPosition(Elevator.PositionPresets.CARGO_LVL_2);
        } else if(mControlBoard.getCargo3Preset()) { 
            mElevator.setPosition(Elevator.PositionPresets.CARGO_LVL_3); 
        } else if(mControlBoard.getCargoCargoShipPreset()) { 
            mElevator.setPosition(Elevator.PositionPresets.CARGO_SHIP); 
        } else if(mControlBoard.getHatch1Preset()) { 
            mElevator.setPosition(Elevator.PositionPresets.HATCH_LVL_1); 
        } else if(mControlBoard.getHatch2Preset()) { 
            mElevator.setPosition(Elevator.PositionPresets.HATCH_LVL_2); 
        } else if(mControlBoard.getHatch3Preset()) { 
            mElevator.setPosition(Elevator.PositionPresets.HATCH_LVL_3);
        }
    }    

    public void climberRoutine() {
        if (mControlBoard.getClimberDeploy()) {
            if (!mClimberIsDeployed) {
                mClimberIsDeployed = true;
                mClimberDeploy.set(Value.kForward);
            }
        } else if (mClimberIsDeployed) {
            mClimberIsDeployed = false;
            mClimberDeploy.set(Value.kReverse);
        }
    }

    public void testRoutines() {
        collectorTestRoutine();
        driveTestRoutine();
        elevatorTestRoutine();
        hatchTestRoutine();
        shooterTestRoutine();
        outputToSmartDashboard();
    }

    private void collectorTestRoutine(){

        if (mControlBoard.getTest_PovLatched(90)){
            // start self test, it will stop on its own or when button is released
            mCollector.setWantedState(Collector.WantedState.SELF_TEST);
        } 
        else if (mControlBoard.getTest_PovLatchedFalse(90)){
            // stop self test by releasing button
            mCollector.setWantedState(Collector.WantedState.BRING_UP);
        }

        Collector.mCollectorTest.setElbowSpin(mControlBoard.getTest_LeftThumbY());
        Collector.mCollectorTest.setIntakeSpin(mControlBoard.getTest_LeftThumbX());
        if (mControlBoard.getTest_Start()){
            Collector.mCollectorTest.setCollectorDeploy(Collector.PhysicalSolenoidState.RETRACTED);
        }
        if (mControlBoard.getTest_Back()){
            Collector.mCollectorTest.setCollectorDeploy(Collector.PhysicalSolenoidState.EXTENDED);
        }
    }

    private void driveTestRoutine(){

        if (mControlBoard.getTest_PovLatched(180)){
            // start self test, it will stop on its own or when button is released
            mDrive.setWantedState(Drive.WantedState.SELF_TEST);
        } 
        else if (mControlBoard.getTest_PovLatchedFalse(180)){
            // stop self test by releasing button
            mDrive.setWantedState(Drive.WantedState.BRING_UP);
        }

        // low gear
        if (mControlBoard.getTest_RightBumper()) {
            mDrive.setHighGear(false);
        }

        // high gear
        if (mControlBoard.getTest_LeftBumper()) {
            mDrive.setHighGear(true);
        }

        // run wheels both forward or both back
        double speed = mControlBoard.getTest_RightTrigger();
        if ( speed > .1){
            Drive.mDriveTest.setSpeed(speed);
        }
        else {
            speed = mControlBoard.getTest_LeftTrigger();
            if ( speed > .1){
                Drive.mDriveTest.setSpeed(-speed);
            }
            else{
                Drive.mDriveTest.setSpeed(0);
            }
        }
    }
    
    public void elevatorTestRoutine() {
        if (mControlBoard.getTest_PovLatched(270)){
            // start self test, it will stop on its own or when button is released
            mElevator.setWantedState(Elevator.WantedState.SELF_TEST);
        } 
        else if (mControlBoard.getTest_PovLatchedFalse(270)){
            // stop self test by releasing button
            mElevator.setWantedState(Elevator.WantedState.BRING_UP);
        }

        Elevator.mElevatorTest.setElevatorMove(mControlBoard.getTest_RightThumbY());
    }

    private void hatchTestRoutine(){
        if (mControlBoard.getTest_PovLatched(0)){
            // start self test, it will stop on its own or when button is released
            mHatchIntake.setWantedState((HatchIntake.WantedState.SELF_TEST));
            // starting the shooter tests with hatch tests is on purpose
            // it is a limitation of the latched code and a limitation on buttons
            mShooter.setWantedState(Shooter.WantedState.SELF_TEST);
        } 
        else if (mControlBoard.getTest_PovLatchedFalse(0)){
            // stop self test by releasing button
            mHatchIntake.setWantedState(HatchIntake.WantedState.BRING_UP);
            mShooter.setWantedState(Shooter.WantedState.BRING_UP);
        }

        if (mControlBoard.getTest_A()){
            HatchIntake.mHatchTest.setBeak(HatchIntake.BeakSolenoidState.CLOSE);
        } else if (mControlBoard.getTest_B()){
            HatchIntake.mHatchTest.setBeak(HatchIntake.BeakSolenoidState.OPEN);
        } else if (mControlBoard.getTest_X()){
            HatchIntake.mHatchTest.setPiston(HatchIntake.EjectSolenoidState.RETRACT);
        } else if (mControlBoard.getTest_Y()){
            HatchIntake.mHatchTest.setPiston(HatchIntake.EjectSolenoidState.EJECT);
        }
    }

    public void shooterTestRoutine() {
        Shooter.mShooterTest.setShooterSpin(mControlBoard.getTest_RightThumbX());
    }

    private int restrictor = 0;
    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();
    }

    private void startLogging() {
        mDrive.startLogging();
        mElevator.startLogging();
        mCollector.startLogging();
    }
}
