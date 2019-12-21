package com.team4911.frc2019.controlboard;

import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.LatchedBooleanFalse;
import com.team254.lib.util.LatchedInt;
import com.team254.lib.util.LatchedIntFalse;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class ControllerWrapper {

    Joystick stick;
    int joystickNum;

    int buttonCount = 0;
    int axisCount = 0;
    int povCount = 0;

    LatchedBoolean[] button_lbs;
    LatchedBooleanFalse[] button_lbfs;
    LatchedBoolean[] axis_lbs;
    LatchedBooleanFalse[] axis_lbfs;
    LatchedInt[][] pov_lis;
    LatchedIntFalse[][] pov_lifs;

    double nextCheck;
    final double timePeriod = .5;

    public ControllerWrapper(int joystickNum){
        this.joystickNum = joystickNum;
        nextCheck = Timer.getFPGATimestamp()+timePeriod;
        initialize();
    }

    private void initialize(){
        stick = new Joystick(joystickNum);
        if (!initPart2()){
            System.out.println("Joystick("+joystickNum+") not found - buttons "+buttonCount+" , axes "+axisCount+" , pov "+povCount);
        }
    }

    private boolean initPart2(){

        buttonCount = stick.getButtonCount();
        axisCount = stick.getAxisCount();
        povCount = stick.getPOVCount();

        if (buttonCount > 0) {
            // buttons start counting at 1
            button_lbs = new LatchedBoolean[buttonCount+1];
            for (int i=0; i<button_lbs.length; i++){
                button_lbs[i] = new LatchedBoolean();
            }

            button_lbfs = new LatchedBooleanFalse[buttonCount+1];
            for (int i=0; i<button_lbs.length; i++){
                button_lbfs[i] = new LatchedBooleanFalse();
            }

            axis_lbs = new LatchedBoolean[axisCount];
            for (int i=0; i<axis_lbs.length; i++){
                axis_lbs[i] = new LatchedBoolean();
            }

            axis_lbfs = new LatchedBooleanFalse[axisCount+1];
            for (int i=0; i<axis_lbs.length; i++){
                axis_lbfs[i] = new LatchedBooleanFalse();
            }

            pov_lis = new LatchedInt[povCount][8];
            for (int i=0; i<pov_lis.length; i++){
                for (int j=0; j<8; j++){
                    pov_lis[i][j] = new LatchedInt();
                }
            }

            pov_lifs = new LatchedIntFalse[povCount][8];
            for (int i=0; i<pov_lifs.length; i++){
                for (int j=0; j<8; j++){
                    pov_lifs[i][j] = new LatchedIntFalse();
                }
            }

            System.out.println("Joystick("+joystickNum+") found - buttons "+buttonCount+" , axes "+axisCount+" , pov "+povCount);
            return true;
        }
        return false;
    }

    public boolean getButtonLatched (int buttonNum){
        if (buttonNum <= buttonCount){
            return button_lbs[buttonNum].update(stick.getRawButton(buttonNum));
        }
        else {
            checkForStick();
        }
        return false;
    }

    public boolean getButtonLatchedFalse (int buttonNum){
        if (buttonNum <= buttonCount){
            return button_lbfs[buttonNum].update(stick.getRawButton(buttonNum));
        }
        else {
            checkForStick();
        }
        return false;
    }

    public boolean getButtonRaw (int buttonNum){
        if (buttonNum < buttonCount){
            return stick.getRawButton(buttonNum);
        }
        else {
            checkForStick();
        }
        return false;
    }

    public double getAxis (int axisNum){
        if (axisNum < axisCount){
            return stick.getRawAxis(axisNum);
        }
        else {
            checkForStick();
        }
        return 0;
    }

    public boolean getAxisLatched (int axisNum){
        if (axisNum < axisCount){
            return axis_lbs[axisNum].update(stick.getRawAxis(axisNum) > .5);
        }
        else {
            checkForStick();
        }
        return false;
    }

    public boolean getAxisLatchedFalse (int axisNum){
        if (axisNum < axisCount){
            return axis_lbfs[axisNum].update(stick.getRawAxis(axisNum) > .5);
        }
        else {
            checkForStick();
        }
        return false;
    }

    public int getPOV (int povNum){
        if (povNum < povCount){
            return stick.getPOV(povNum);
        }
        else {
            checkForStick();
        }
        return -1;
    }

    public boolean getPOVLatched (int povNum, int desiredValue){
        if (povNum < povCount){
            // desired value is an angle from 0 to 315 in increments of 45
            int angle = desiredValue/45; // angle is an index into the latched pov array
            if (angle < pov_lis[povNum].length){
                return pov_lis[povNum][angle].update(stick.getPOV(povNum),desiredValue);
            }
        }
        else {
            checkForStick();
        }
        return false;
    }

    public boolean getPOVLatchedFalse (int povNum, int desiredValue){
        if (povNum < povCount){
            // desired value is an angle from 0 to 315 in increments of 45
            int angle = desiredValue/45; // angle is an index into the latched pov array

            if (angle < pov_lifs[povNum].length){
                return pov_lifs[povNum][angle].update(stick.getPOV(povNum),desiredValue);
            }
        }
        else {
            checkForStick();
        }
        return false;
    }

    private void checkForStick(){
        if (Timer.getFPGATimestamp() > nextCheck){
            initPart2();
            nextCheck += timePeriod;
        }
    }
}