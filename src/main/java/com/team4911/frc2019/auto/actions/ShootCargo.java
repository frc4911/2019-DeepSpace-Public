package com.team4911.frc2019.auto.actions;

import com.team254.lib.autos.actions.Action;
import com.team4911.frc2019.subsystems.Shooter;
import com.team4911.frc2019.subsystems.Shooter.WantedState;

import edu.wpi.first.wpilibj.Timer;

public class ShootCargo implements Action {
    
    Shooter mShooter = Shooter.getInstance();
    double timeout = 0.0;
    final double kWaitTime = 3.0;

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= timeout;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mShooter.setWantedState(WantedState.HOLD);
    }

    @Override
    public void start() {
        timeout = Timer.getFPGATimestamp() + kWaitTime;
        mShooter.setWantedState(WantedState.SHOOT);
    }
}
