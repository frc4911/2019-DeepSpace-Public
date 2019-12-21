package com.team4911.frc2019.auto.creators;

import com.team254.lib.autos.AutoModeBase;
import com.team4911.frc2019.auto.AutoModeSelector.PlatformPosition;
import com.team4911.frc2019.auto.AutoModeSelector.StartingPosition;
import com.team4911.frc2019.auto.modes.EmptyMode;
import com.team4911.frc2019.auto.modes.LeftLvl1ToLeftShipTwoHatchMode;
import com.team4911.frc2019.auto.modes.RightLvl1ToRightShipTwoHatchMode;

public class ShipTwoHatchModeCreator implements AutoModeCreator {

    private LeftLvl1ToLeftShipTwoHatchMode mLvl1LeftMode = new LeftLvl1ToLeftShipTwoHatchMode();
    private RightLvl1ToRightShipTwoHatchMode mLvl1RightMode = new RightLvl1ToRightShipTwoHatchMode();
    private EmptyMode mEmptyMode = new EmptyMode();

    private StartingPosition position;
    private PlatformPosition level;
    private AutoModeBase mAutoMode;

    public ShipTwoHatchModeCreator(StartingPosition position, PlatformPosition level) {
        this.position = position;
        this.level = level;
    }

    @Override
    public AutoModeBase getAutoMode() {
        if(level == PlatformPosition.LEVEL_ONE) {
            if(position == StartingPosition.LEFT) {
                mAutoMode = mLvl1LeftMode;
            } else if(position == StartingPosition.CENTER) {
                mAutoMode = mEmptyMode;
            } else if(position == StartingPosition.RIGHT) {
                mAutoMode = mLvl1RightMode;
            }
        } else if(level == PlatformPosition.LEVEL_TWO) {
            if(position == StartingPosition.LEFT) {
                mAutoMode = mEmptyMode;
            } else if(position == StartingPosition.CENTER) {
                mAutoMode = mEmptyMode;
            } else if(position == StartingPosition.RIGHT) {
                mAutoMode = mEmptyMode;
            }
        }

        return mAutoMode;
    }

}