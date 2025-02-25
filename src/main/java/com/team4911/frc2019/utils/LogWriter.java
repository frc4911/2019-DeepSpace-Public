package com.team4911.frc2019.utils;

import com.team254.lib.subsystems.SubsystemManager;
import com.team254.lib.util.CrashTrackingRunnable;

public class LogWriter {
    private SubsystemManager mSubsystemManager = null;
    private Thread mThread = null;
    private int mThreadPriority = Thread.NORM_PRIORITY;
    private long mWaitTime;
    private volatile boolean mIsRunning;

    public void registerSubsystemManager(SubsystemManager subsystemManager, long waitTime, int threadPriority) {
        mSubsystemManager = subsystemManager;
        mWaitTime = waitTime;
        if (threadPriority == 0) {
            mThreadPriority = Thread.NORM_PRIORITY;
        } else {
            mThreadPriority = Math.max(Math.min(Thread.MAX_PRIORITY, threadPriority), Thread.MIN_PRIORITY);
        }

        if (mSubsystemManager != null) {
            mThread = new Thread(new CrashTrackingRunnable() {
                @Override
                public void runCrashTracked() {
                    if (mSubsystemManager != null) {
                        while (mIsRunning) {
                            mSubsystemManager.writeToLog();
                            try {
                                Thread.sleep(mWaitTime);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                                Thread.currentThread().interrupt();
                            }
                        }
                    }
                }
            });
        }
    }

    public void start() {
        if (mThread != null && !mThread.isAlive()) {
            System.out.println("Starting LogWriter");
            mThread.setPriority(mThreadPriority);
            mIsRunning = true;
            mThread.start();
        }
    }

    public void stop() {
        if (mSubsystemManager != null) {
            mSubsystemManager.writeToLog();
            mIsRunning = false;
        }
    }
}
