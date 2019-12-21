package com.team4911.frc2019.controlboard;

public class ControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }
        return mInstance;
    }

    private LeftDriveController leftDriveController;
    private RightDriveController rightDriveController;
    private OperatorController operatorController;
    private LogitechPS4 testController;

    private ControlBoard() {
        leftDriveController = new LeftDriveController(0);
        rightDriveController = new RightDriveController(1);
        operatorController = new OperatorController(2);
        testController = new LogitechPS4(3);
    }

    public double getThrottleRight() {
        return rightDriveController.getThrottle();
    }

    public double getThrottleLeft() {
        return leftDriveController.getThrottle();
    }

    public double getTurn() {
        return leftDriveController.getTurn();
    }

    public boolean getLowGear() {
        return leftDriveController.getLowGear();
    }

    public boolean getHighGear() {
        return rightDriveController.getHighGear();
    }

    public boolean getInfoDump() {
        return leftDriveController.getInfoDump();
    }

    public boolean getClimberDeploy() {
        return leftDriveController.getClimberDeploy();
    }

    public boolean getDriveChoice() {
        return rightDriveController.getDriveChoice();
    }

    // public boolean getAutoPilot() { return leftDriveController.getAutoPilot(); }

    public boolean getDriveBringUp() {
        return rightDriveController.getDriveBringUp();
    }

    public boolean getDriveSelfTestPressed() {
        return rightDriveController.getDriveSelfTestPressed();
    }

    public boolean getDriveSelfTestReleased() {
        return rightDriveController.getDriveSelfTestReleased();
    }

    public boolean getShooterBringUp() {
        return rightDriveController.getShooterBringUp();
    }

    public boolean getShooterSelfTestPressed() {
        return rightDriveController.getShooterSelfTestPressed();
    }

    public boolean getShooterSelfTestReleased() {
        return rightDriveController.getShooterSelfTestReleased();
    }

    public boolean getCollectorBringUp() {
        return rightDriveController.getCollectorBringUp();
    }

    public boolean getCollectorSelfTestPressed() {
        return rightDriveController.getCollectorSelfTestPressed();
    }

    public boolean getCollectorSelfTestReleased() {
        return rightDriveController.getCollectorSelfTestReleased();
    }

    public boolean getElevatorBringUp() {
        return rightDriveController.getElevatorBringUp();
    }

    public boolean getElevatorSelfTestPressed() {
        return rightDriveController.getElevatorSelfTestPressed();
    }

    public boolean getElevatorSelfTestReleased() {
        return rightDriveController.getElevatorSelfTestReleased();
    }

    public boolean getHatchIntakeBringUp() {
        return rightDriveController.getHatchIntakeBringUp();
    }

    public boolean getHatchIntakeSelfTestPressed() {
        return rightDriveController.getHatchIntakeSelfTestPressed();
    }

    public boolean getHatchIntakeSelfTestReleased() {
        return rightDriveController.getHatchIntakeSelfTestReleased();
    }

    public boolean getCollectCargo() {
        if(operatorController.getCollectCargo() > 0.5) {
            return true;
        }
        return false;
    }

    public boolean getShootCargo() {
        if(operatorController.getShootCargo() > 0.5) {
            return true;
        }
        return false;
    }

    public boolean getEjectHatch() {
        return operatorController.getEjectHatch();
    }

    public boolean getRetractHatch() {
        return operatorController.getRetractHatch();
    }

    public boolean getCollectHatch() {
        return operatorController.getCollectHatch();
    }

    public boolean getStowHatch() {
        return operatorController.getStowHatch();
    }

    public double getMoveElevator() {
        return operatorController.getMoveElevator();
    }

    public double getMoveRandomThing() {// Test Code
        return operatorController.getMoveRandomThing();// Test Code
    }// Test Code

    public boolean getHatch1Preset() {
        return operatorController.getHatch1Preset();
    }

    public boolean getHatch2Preset() {
        return operatorController.getHatch2Preset();
    }

    public boolean getHatch3Preset() {
        return operatorController.getHatch3Preset();
    }

    public boolean getCollectCargoPreset() {
        return operatorController.getCollectCargoPreset();
    }

    public boolean getCargo1Preset() {
        return operatorController.getCargo1Preset();
    }

    public boolean getCargo2Preset() {
        return operatorController.getCargo2Preset();
    }

    public boolean getCargo3Preset() {
        return operatorController.getCargo3Preset();
    }

    public boolean getCargoCargoShipPreset() {
        return operatorController.getCargoCargoShipPreset();
    }

    public boolean getClimberUnclamp() {
        return operatorController.getClimberUnclamp();
    }

    // test mode user interface
    public boolean getTest_A() {
        return testController.getButtonA();
    }

    public boolean getTest_B() {
        return testController.getButtonB();
    }

    public boolean getTest_X() {
        return testController.getButtonX();
    }

    public boolean getTest_Y() {
        return testController.getButtonY();
    }

    public boolean getTest_Start() {
        return testController.getButtonStart();
    }

    public boolean getTest_Back() {
        return testController.getButtonBack();
    }

    public double getTest_LeftThumbY() {
        return testController.getLeftStickY();
    }

    public double getTest_RightThumbY() {
        return testController.getRightStickY();
    }

    public double getTest_LeftThumbX() {
        return testController.getLeftStickX();
    }

    public double getTest_RightThumbX() {
        return testController.getRightStickX();
    }

    public boolean getTest_LeftBumper() {
        return testController.getLeftBumper();
    }

    public boolean getTest_RightBumper() {
        return testController.getRightBumper();
    }

    public double getTest_LeftTrigger() {
        return testController.getLeftTrigger();
    }

    public double getTest_RightTrigger() {
        return testController.getRightTrigger();
    }

    public boolean getTest_Pov(int angle) {
        return testController.getPov(angle);
    }

    public int getTest_Pov() {
        return testController.getPov();
    }

    public boolean getTest_PovLatched(int angle) {
        return testController.getPovLatched(angle);
    }

    public boolean getTest_PovLatchedFalse(int angle) {
        return testController.getPovLatchedFalse(angle);
    }
}
