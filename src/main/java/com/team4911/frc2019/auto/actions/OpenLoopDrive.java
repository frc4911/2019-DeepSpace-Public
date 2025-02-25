package com.team4911.frc2019.auto.actions;

import com.team4911.frc2019.subsystems.Drive;
import com.team254.lib.autos.actions.Action;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

public class OpenLoopDrive implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private double mStartTime;
    private final double mDuration, mLeft, mRight;
    private final boolean mFinishWhenSeesCube;

    public OpenLoopDrive(double left, double right, double duration, boolean finishWhenSeesCube) {
        mDuration = duration;
        mLeft = left;
        mRight = right;
        mFinishWhenSeesCube = finishWhenSeesCube;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > mDuration || mFinishWhenSeesCube;
    }

    @Override
    public void update() {
        // System.out.println((Timer.getFPGATimestamp() - mStartTime) + " > " + mDuration);

    }

    @Override
    public void done() {
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal(mLeft, mRight));
        mStartTime = Timer.getFPGATimestamp();
    }
}
