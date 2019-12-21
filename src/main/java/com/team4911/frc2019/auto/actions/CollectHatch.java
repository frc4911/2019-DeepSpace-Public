package com.team4911.frc2019.auto.actions;

import com.team254.lib.autos.actions.Action;
import com.team4911.frc2019.subsystems.HatchIntake;
import com.team4911.frc2019.subsystems.HatchIntake.WantedState;

import edu.wpi.first.wpilibj.Timer;

public class CollectHatch implements Action {
    
    HatchIntake mHatchIntake = HatchIntake.getInstance();
    boolean collect;
    double timeout = 0.0;
    final double kWaitTime = 0.3;

    public CollectHatch(boolean collect) {
        this.collect = collect;
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
        
        if(collect) {
            mHatchIntake.setWantedState(WantedState.OPEN);
        } else {
            mHatchIntake.setWantedState(WantedState.STOW);
        }
    }
}
