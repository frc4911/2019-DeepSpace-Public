package com.team4911.frc2019.auto.creators;

import com.team254.lib.autos.AutoModeBase;
import com.team4911.frc2019.auto.AutoModeSelector.PlatformPosition;
import com.team4911.frc2019.auto.AutoModeSelector.StartingPosition;
import com.team4911.frc2019.auto.modes.LeftLvl1ToLeftRocketTwoHatchNearMode;

public class RocketTwoHatchNearModeCreator implements AutoModeCreator {
	
	private LeftLvl1ToLeftRocketTwoHatchNearMode mLvl1LeftMode = new LeftLvl1ToLeftRocketTwoHatchNearMode();
    
    private StartingPosition position;
    private PlatformPosition level;
    private AutoModeBase mAutoMode;

    public RocketTwoHatchNearModeCreator(StartingPosition position, PlatformPosition level) {
        this.position = position;
        this.level = level;
    }

	@Override
    public AutoModeBase getAutoMode() {
        if(level == PlatformPosition.LEVEL_ONE) {
            if(position == StartingPosition.LEFT) {
                mAutoMode = mLvl1LeftMode;
            } else if(position == StartingPosition.CENTER) {
                
            } else if(position == StartingPosition.RIGHT) {

            }
        } else if(level == PlatformPosition.LEVEL_TWO) {
            if(position == StartingPosition.LEFT) {

            } else if(position == StartingPosition.RIGHT) {

            }
        }

        return mLvl1LeftMode; //mAutoMode;
    }
}