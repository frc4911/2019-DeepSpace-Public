package com.team4911.frc2019.controlboard;

public class LogitechPS4 {

    ControllerWrapper controller;

    public LogitechPS4(int stickNum){
        controller = new ControllerWrapper(stickNum);
    }

    public double getLeftTrigger(){
        return controller.getAxis(2);
    }

    public double getRightTrigger(){
        return controller.getAxis(3);
    }

    public boolean getLeftBumper() {
        return controller.getButtonLatched(5);
    }

    public boolean getLeftBumperReleased() {
        return controller.getButtonLatchedFalse(5);
    }

    public boolean getRightBumper(){
        return controller.getButtonLatched(6);
    }

    public boolean getRightBumperReleased(){
        return controller.getButtonLatchedFalse(6);
    }

    public double getLeftStickY(){
        return -controller.getAxis(1);
    }

    public double getRightStickY(){
        return -controller.getAxis(5);
    }

    public double getLeftStickX(){
        return controller.getAxis(0);
    }

    public double getRightStickX(){
        return controller.getAxis(4);
    }

    public int getPov() {
        return controller.getPOV(0);
    }

    public boolean getPov(int angle) {
        if(controller.getPOV(0) == angle) {
            return true;
        }
        return false;
    }

    public boolean getPovLatched(int angle){
        return controller.getPOVLatched(0, angle);
    }

    public boolean getPovLatchedFalse(int angle) {
        return controller.getPOVLatchedFalse(0, angle);
    }

    public boolean getButtonA(){
        return controller.getButtonLatched(1);
    }

    public boolean getButtonAReleased(){
        return controller.getButtonLatchedFalse(1);
    }

    public boolean getButtonX(){
        return controller.getButtonLatched(3);
    }

    public boolean getButtonXReleased(){
        return controller.getButtonLatchedFalse(3);
    }

    public boolean getButtonY(){
        return controller.getButtonLatched(4);
    }

    public boolean getButtonYReleased(){
        return controller.getButtonLatchedFalse(4);
    }

    public boolean getButtonB(){
        return controller.getButtonLatched(2);
    }

    public boolean getButtonBReleased(){
        return controller.getButtonLatchedFalse(2);
    }

    public boolean getButtonStart(){
        return controller.getButtonRaw(8);
    }

    public boolean getButtonBack(){
        return controller.getButtonRaw(7);
    }
}