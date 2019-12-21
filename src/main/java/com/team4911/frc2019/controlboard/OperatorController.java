package com.team4911.frc2019.controlboard;

import com.team4911.frc2019.utils.Annot4911;

public class OperatorController {

    ControllerWrapper controller;

    @Annot4911(description = "Operator Controller")
    public OperatorController(int stickNum){
        controller = new ControllerWrapper(stickNum);
    }

    @Annot4911(description = "Left Trigger - collect cargo")
    public double getCollectCargo(){
        return controller.getAxis(2);
    }

    @Annot4911(description = "Right Trigger - shoot cargo")
    public double getShootCargo(){
        return controller.getAxis(3);
    }

    @Annot4911(description = "Left Bumper - collect hatch")
    public boolean getCollectHatch() {
        return controller.getButtonLatched(5);
    }

    public boolean getStowHatch(){
        return controller.getButtonLatchedFalse(5);
    }
  
    @Annot4911(description = "Right Bumper - eject hatch")
    public boolean getEjectHatch(){
        return controller.getButtonLatched(6);
    }

    public boolean getRetractHatch(){
        return controller.getButtonLatchedFalse(6);
    }

    @Annot4911(description = "Left Stick - move elevator")
    public double getMoveElevator(){
        return -controller.getAxis(1);
    }

    @Annot4911(description = "Right Stick - TBD") // Test Code
    public double getMoveRandomThing(){// Test Code
        return -controller.getAxis(2);// Test Code
    }// Test Code

    @Annot4911(description = "D-Pad Down - hatch preset lvl 1")
    public boolean getHatch1Preset() {
        if(controller.getPOV(0) == 180) {
            return true;
        }
        return false;
    }

    @Annot4911(description = "D-Pad Left - hatch preset lvl 2")
    public boolean getHatch2Preset() {
        if(controller.getPOV(0) == 270) {
            return true;
        }
        return false;
    }

    @Annot4911(description = "D-Pad Up - hatch preset lvl 3")
    public boolean getHatch3Preset() {
        if(controller.getPOV(0) == 0) {
            return true;
        }
        return false;
    }

    @Annot4911(description = "D-Pad Right - collect preset")
    public boolean getCollectCargoPreset() {
        if(controller.getPOV(0) == 90) {
            return true;
        }
        return false;
    }

    @Annot4911(description = "A - cargo preset lvl 1")
    public boolean getCargo1Preset(){
        return controller.getButtonLatched(1);
    }

    @Annot4911(description = "X - cargo preset lvl 2")
    public boolean getCargo2Preset(){
        return controller.getButtonLatched(3);
    }

    @Annot4911(description = "Y - cargo preset lvl 3")
    public boolean getCargo3Preset(){
        return controller.getButtonLatched(4);
    }

    @Annot4911(description = "B - cargo preset cargoShip")
    public boolean getCargoCargoShipPreset(){
        return controller.getButtonLatched(2);
    }

    @Annot4911(description = "Start - deploy climber")
    public boolean getClimberDeploy(){
        return controller.getButtonRaw(8);
    }

    @Annot4911(description = "Back - climber unclamp")
    public boolean getClimberUnclamp(){
        return controller.getButtonRaw(7);
    }
} 