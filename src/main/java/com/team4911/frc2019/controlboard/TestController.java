package com.team4911.frc2019.controlboard;

import com.team4911.frc2019.utils.Annot4911;

public class TestController {

    ControllerWrapper controller;

    @Annot4911(description = "Test Controller")
    public TestController(int stickNum){
        controller = new ControllerWrapper(stickNum);
    }

    @Annot4911(description = "Left Trigger - Test")
    public double getLeftTrigger(){
        return controller.getAxis(2);
    }

    @Annot4911(description = "Right Trigger - Test")
    public double getRightTrigger(){
        return controller.getAxis(3);
    }

    @Annot4911(description = "Left Bumper - Test")
    public boolean getLeftBumper() {
        return controller.getButtonLatched(5);
    }

    @Annot4911(description = "Right Bumper - Test")
    public boolean getRightBumper(){
        return controller.getButtonLatched(6);
    }

    @Annot4911(description = "Left Stick Y - Test")
    public double getLeftStickY(){
        return -controller.getAxis(1);
    }

    @Annot4911(description = "Right Stick Y - Test")
    public double getRightStickY(){
        return -controller.getAxis(5);
    }

    @Annot4911(description = "Left Stick X - Test")
    public double getLeftStickX(){
        return controller.getAxis(0);
    }

    @Annot4911(description = "Right Stick X - Test")
    public double getRightStickX(){
        return controller.getAxis(4);
    }

    @Annot4911(description = "D-Pad Down - Test")
    public boolean getDPadDown() {
        if(controller.getPOV(0) == 180) {
            return true;
        }
        return false;
    }

    @Annot4911(description = "D-Pad Left - Test")
    public boolean getDPadLeft() {
        if(controller.getPOV(0) == 270) {
            return true;
        }
        return false;
    }

    @Annot4911(description = "D-Pad Up - Test")
    public boolean getDPadUp() {
        if(controller.getPOV(0) == 0) {
            return true;
        }
        return false;
    }

    @Annot4911(description = "D-Pad Right - Test")
    public boolean getDPadRight() {
        if(controller.getPOV(0) == 90) {
            return true;
        }
        return false;
    }

    @Annot4911(description = "A - Test")
    public boolean getButtonA(){
        return controller.getButtonLatched(1);
    }

    @Annot4911(description = "X - Test")
    public boolean getButtonX(){
        return controller.getButtonLatched(3);
    }

    @Annot4911(description = "Y - Test")
    public boolean getButtonY(){
        return controller.getButtonLatched(4);
    }

    @Annot4911(description = "B - Test")
    public boolean getButtonB(){
        return controller.getButtonLatched(2);
    }

    @Annot4911(description = "Start - Test")
    public boolean getButtonStart(){
        return controller.getButtonRaw(8);
    }

    @Annot4911(description = "Back - Test")
    public boolean getButtonBack(){
        return controller.getButtonLatched(7);
    }
} 