package com.team4911.frc2019.controlboard;

import com.team4911.frc2019.utils.Annot4911;

public class LeftDriveController {

    ControllerWrapper controller;

    @Annot4911(description = "USB 0 - Left Drive Joystick")
    public LeftDriveController(int stickNum){
        controller = new ControllerWrapper(stickNum);
    }

    @Annot4911(description = "Trigger - quick turn")
    public boolean getLowGear(){
        return controller.getButtonLatched(1);
    }

    @Annot4911(description = "Trigger - info dump (disabled mode only)")
    public boolean getInfoDump(){
        return controller.getButtonLatched(1);
    }

    @Annot4911(description = "Y-axis - throttle (Tank)")
    public double getThrottle(){
        return controller.getAxis(1);
    }

    @Annot4911(description = "X-axis - turn (Cheesy, Arcade)")
    public double getTurn(){
        return controller.getAxis(0);
    }

    public boolean getClimberDeploy() {
        return controller.getButtonRaw(2);
    }

    // start selftest when pressed
    @Annot4911(description = "Trigger - Drive SelfTest (Test mode only)")
    public boolean getDriveSelfTestLatched(){
        return controller.getButtonLatched(1);
    }

    // no annotation needed
    // stop selftest when released
    public boolean getDriveSelfTestLatchedFalse(){
        return controller.getButtonLatchedFalse(1);
    }

    @Annot4911(description = "Trigger - Autopilot to a target")
    public boolean getAutoPilot() { return controller.getButtonLatched(2); }
}