package com.team4911.frc2019.auto.actions;

import com.team254.lib.autos.actions.Action;
import com.team4911.frc2019.subsystems.HatchIntake;
import com.team4911.frc2019.subsystems.HatchIntake.WantedState;

import edu.wpi.first.wpilibj.Timer;

public class ShootHatch implements Action {
    
    HatchIntake mHatchIntake = HatchIntake.getInstance();
    boolean shoot;
    double timeout = 0.0;
    final double kWaitTime = 0.5;

    public ShootHatch(boolean shoot) {
        this.shoot = shoot;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= timeout;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
        timeout = Timer.getFPGATimestamp() + kWaitTime;

        if(shoot) {
            mHatchIntake.setWantedState(WantedState.OPEN_AND_EJECT);
        } else {
            mHatchIntake.setWantedState(WantedState.STOW);
        }
    }
}
