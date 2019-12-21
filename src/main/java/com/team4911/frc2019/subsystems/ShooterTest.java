package com.team4911.frc2019.subsystems;

import com.team254.lib.util.RobotName;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.subsystems.Shooter.SystemState;
import com.team4911.frc2019.utils.TestValueTracker;

public class ShooterTest {

    private Shooter mShooter;

    public ShooterTest() {
        this.mShooter = Shooter.getInstance();
    }

    // how can we notify the tester if the shooter is going correct dir
    public SystemState handleBringUp() {
        return mShooter.defaultStateTransfer();
    }

    public synchronized void setShooterSpin(double speed) {
        if(mShooter.mSystemState == SystemState.BRINGING_UP) {
            mShooter.mPeriodicIO.demand = speed;
        }
    }

    private TestResults mTestResults = new TestResults();
    private double mLastSampleTime;
    private double mStartTime;
    private int mCurrentTest = 0;
    private int mPrevTest = -1;

    private class TestResults{

        TestValueTracker[] components = {new TestValueTracker("Shooter talon")};

        public void process(){
            components[0].process(mShooter.mPeriodicIO.current_draw);
        }

        public void dumpResults(){

            System.out.println("---------------------------------------");
            System.out.println("Shooter Test");
            if (RobotName.name.equals(Constants.kSilverName)){
                components[0].dumpMax(3.0, 1.0);
                components[0].dumpAve(1.0, 0.3);
            }
            else {
                components[0].dumpMax(8.5, 2.0);
                components[0].dumpAve(2.7, 1.0);
            }        
        }
    }
    
    private boolean runShooterTest(TestResults tr, boolean start, double now){

        if (start){
            mLastSampleTime = now;
            mStartTime = now;
        }

        if (now - mLastSampleTime >= .1){
            mLastSampleTime = now;
            tr.process();
        }

        // set speed based on time
        double speed = (now-mStartTime)/1.0;  // 1.0 means 1.0 x 4 = 4.0 second test duration

        if (speed < 1){
            // ramp speed from 0.0 to 1.0
        } else if (speed < 3.0){
            // ramp speed from 1.0 to - 1.0
            speed = 2.0 - speed;
        } else if (speed < 4.0) {
            // ramp speed from -1.0 to 0
            speed = speed-4.0;
        }

        if (speed >= 4.0) {
            // test over
            tr.dumpResults();
        }
        else {
            mShooter.mPeriodicIO.demand = speed;
        }

        return speed >= 4.0;
    }

    private boolean runAllShooterTests(boolean firstTime, double now){

        if (firstTime){
            mCurrentTest = 0;
            mPrevTest = -1;
        }

        boolean newStart = mPrevTest != mCurrentTest;
        boolean allDone = false;

        if (newStart){
            switch (mCurrentTest){
                case 0:
                    mTestResults = new TestResults();
                    break;
                default:
                    allDone = true;
                    break;
            }
            mPrevTest = mCurrentTest;
        }

        if(!allDone){
            if (runShooterTest(mTestResults, newStart, now)){
                mCurrentTest++;
            }
        }
        
        return allDone;
    }

    public Shooter.SystemState handleSelfTesting(double now){
        if (runAllShooterTests(mShooter.mStateChanged, now)){
            mShooter.setWantedState(Shooter.WantedState.BRING_UP);
        }
        return mShooter.defaultStateTransfer();
    }
}