package com.team4911.frc2019.auto.creators;

import com.team254.lib.autos.AutoModeBase;
import com.team4911.frc2019.auto.modes.DriveBackwardsMode;

public class DriveBackwardsModeCreator implements AutoModeCreator {
	
	private DriveBackwardsMode mAutoMode = new DriveBackwardsMode();

	@Override
    public AutoModeBase getAutoMode() {
        return mAutoMode;
    }
}