package com.team4911.frc2019.auto.actions;

import com.team254.lib.autos.actions.Action;
import com.team4911.frc2019.subsystems.Elevator;
import com.team4911.frc2019.subsystems.Elevator.PositionPresets;
import com.team4911.frc2019.subsystems.Elevator.WantedState;

public class MoveElevatorToPreset implements Action {
    
    Elevator mElevator = Elevator.getInstance();
    PositionPresets preset;
    
    public MoveElevatorToPreset(PositionPresets preset) {
        this.preset = preset;
    }

    @Override
    public boolean isFinished() {
        return mElevator.getWantedState() == WantedState.HOLD;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        mElevator.setPosition(preset);
    }
}
