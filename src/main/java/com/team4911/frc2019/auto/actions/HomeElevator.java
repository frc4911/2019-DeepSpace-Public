package com.team4911.frc2019.auto.actions;

import com.team254.lib.autos.actions.Action;
import com.team4911.frc2019.subsystems.Elevator;
import com.team4911.frc2019.subsystems.Elevator.WantedState;

public class HomeElevator implements Action {
    
    Elevator mElevator = Elevator.getInstance();

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mElevator.setWantedState(WantedState.HOME);
    }
}
