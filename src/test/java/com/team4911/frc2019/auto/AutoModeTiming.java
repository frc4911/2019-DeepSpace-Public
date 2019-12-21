package com.team4911.frc2019.auto;

import com.team4911.frc2019.paths.TrajectoryGenerator;
import org.junit.jupiter.api.Test;

public class AutoModeTiming {
    TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    boolean mStartedLeft = true;

    @Test
    void checkTiming() {
        mTrajectoryGenerator.generateTrajectories();
    }
}
