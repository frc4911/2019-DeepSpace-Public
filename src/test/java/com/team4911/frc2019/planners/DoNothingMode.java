package com.team4911.frc2019.planners;

import com.team254.lib.autos.AutoModeBase;
import com.team254.lib.autos.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Doing nothing");
    }
}
