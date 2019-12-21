package com.team4911.frc2019.controlboard;

import com.team4911.frc2019.utils.Annot4911;

public class RightDriveController {

    ControllerWrapper controller;

    @Annot4911(description = "USB 1 - Right Drive Joystick")
    public RightDriveController(int stickNum){
        controller = new ControllerWrapper(stickNum);
    }

    @Annot4911(description = "Y-axis - throttle (all modes)")
    public double getThrottle(){
        return -controller.getAxis(1);
    }

    @Annot4911(description = "Trigger - Shift High Gear")
    public boolean getHighGear(){
        return controller.getButtonLatched(1);
    }

    @Annot4911(description = "Button 2 - drive mode (Arcade, Cheesy, Tank)")
    public boolean getDriveChoice(){
        return controller.getButtonLatched(2);
    }

    @Annot4911(description = "Button 5 - Drive Bring Up Test")
    public boolean getDriveBringUp(){
        return controller.getButtonLatched(5);
    }

    @Annot4911(description = "Button 10 - Drive Self Test")
    public boolean getDriveSelfTestPressed(){
        return controller.getButtonLatched(10);
    }

    public boolean getDriveSelfTestReleased(){
        return controller.getButtonLatchedFalse(10);
    }

    @Annot4911(description = "Button 6 - Shooter Bring Up Test")
    public boolean getShooterBringUp(){
        return controller.getButtonLatched(6);
    }

    @Annot4911(description = "Button 9 - Shooter Self Test")
    public boolean getShooterSelfTestPressed(){
        return controller.getButtonLatched(9);
    }

    public boolean getShooterSelfTestReleased(){
        return controller.getButtonLatchedFalse(9);
    }

    @Annot4911(description = "Button 7 - Collector Bring Up Test")
    public boolean getCollectorBringUp(){
        return controller.getButtonLatched(7);
    }

    @Annot4911(description = "Button 8 - Collector Self Test")
    public boolean getCollectorSelfTestPressed(){
        return controller.getButtonLatched(8);
    }

    public boolean getCollectorSelfTestReleased(){
        return controller.getButtonLatchedFalse(8);
    }

    @Annot4911(description = "Button 13 - HatchIntake Bring Up Test")
    public boolean getHatchIntakeBringUp(){
        return controller.getButtonLatched(13);
    }

    @Annot4911(description = "Button 14 - HatchIntake Self Test")
    public boolean getHatchIntakeSelfTestPressed(){
        return controller.getButtonLatched(14);
    }

    public boolean getHatchIntakeSelfTestReleased(){
        return controller.getButtonLatchedFalse(14);
    }

    @Annot4911(description = "Button 12 - Elevator Bring Up Test")
    public boolean getElevatorBringUp(){
        return controller.getButtonLatched(12);
    }

    @Annot4911(description = "Trigger - drive mode (Cheesy, Arcade, Tank)")
    public boolean getElevatorSelfTestPressed(){
        return controller.getButtonLatched(15);
    }

    public boolean getElevatorSelfTestReleased(){
        return controller.getButtonLatchedFalse(15);
    }
} 
