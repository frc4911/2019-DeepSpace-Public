package com.team4911.frc2019.auto.modes;

import com.team254.lib.autos.AutoModeBase;
import com.team254.lib.autos.AutoModeEndedException;
import com.team4911.frc2019.auto.actions.OpenLoopDrive;

public class DriveBackwardsMode extends AutoModeBase {

	@Override
    protected void routine() throws AutoModeEndedException {
        runAction(new OpenLoopDrive(-0.33, -0.33, 2.0, false));
    }
	
}