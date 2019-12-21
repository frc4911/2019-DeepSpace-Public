package com.team4911.frc2019.auto.actions;

import com.team4911.frc2019.subsystems.Drive;
import com.team254.lib.autos.actions.RunOnceAction;

public class OverrideTrajectory extends RunOnceAction {
    @Override
    public void runOnce() {
        Drive.getInstance().overrideTrajectory(true);
    }
}
