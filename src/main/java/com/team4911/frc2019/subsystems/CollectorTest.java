package com.team4911.frc2019.subsystems;

import com.team254.lib.util.RobotName;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.controlboard.TestController;
import com.team4911.frc2019.subsystems.Collector.PhysicalSolenoidState;
import com.team4911.frc2019.utils.TestValueTracker;

public class CollectorTest {

    Collector mCollector;
    HatchIntake mHatchIntake;

    public CollectorTest (){
        this.mCollector = Collector.getInstance();
        this.mHatchIntake = HatchIntake.getInstance();
    }

    public Collector.SystemState handleBringingUp() {
        return mCollector.defaultStateTransfer();
    }

    public synchronized void setIntakeSpin(double speed) {
        if(mCollector.mSystemState == Collector.SystemState.BRINGING_UP) {
            mCollector.mPeriodicIO.rollerDemand = speed;
        }
    }

    public synchronized void setElbowSpin(double speed) {
        if(mCollector.mSystemState == Collector.SystemState.BRINGING_UP) {
            mCollector.mPeriodicIO.elbowDemand = speed*.2;
        }
    }

    public void setCollectorDeploy(Collector.PhysicalSolenoidState state){
        if(mCollector.mSystemState == Collector.SystemState.BRINGING_UP) {
            mCollector.mPeriodicIO.solenoidDemand = state;
        }
    }

    private TestResults mTestResults = new TestResults("Collector Test");
    private double mLastSampleTime;
    private double mStartTime;
    private int mCurrentTest = 0;
    private int mPrevTest = -1;
    private boolean mSafeToRun = false;

    private class TestResults{

        String description;

        public TestResults(String description){
            this.description = description;
        }

        TestValueTracker[] components = {new TestValueTracker("Intake wheels amps"),
                                        new TestValueTracker("Elbow encoder ticks"),
                                        new TestValueTracker("Elbow amps"),
                                        new TestValueTracker("Solenoid psi"),
                                        new TestValueTracker("Limit Switch Value")}; 

        public void process(){
            components[0].process(mCollector.mPeriodicIO.roller_current);
            components[1].process(mCollector.mPeriodicIO.elbowPosition);
            components[2].process(mCollector.mPeriodicIO.elbow_current);
            components[3].process(mHatchIntake.mPeriodicIO.pneumaticPressure);
            components[4].process(mCollector.mPeriodicIO.limit_switch);
        }

        public void dumpResults(boolean motors){

            System.out.println("------------------------------------");
            System.out.println(description);
            if (RobotName.name.equals(Constants.kSilverName)){
                if (motors){
                    components[0].dumpMax(18, 8.0);
                    components[0].dumpAve(12, 3.0);
                    components[1].dumpMax(0, 50);
                    components[1].dumpMin(-835, 65);
                    components[2].dumpMax(7.5, 2.5);
                    components[2].dumpAve(5, 1);
                }
                else{
                    components[3].dumpChange(3, 1);
                    components[4].dumpLast(1.0, 0);
                }
            }
            else {
                // chrome
                if (motors){
                    components[0].dumpMax(22.5, 5.0);
                    components[0].dumpAve(12.5, 3.0);
                    components[1].dumpMax(0, 50);
                    components[1].dumpMin(-3800, 100);
                    components[2].dumpMax(9.0, 2.5);
                    components[2].dumpAve(5.5, 1);
                }
                else{
                    components[3].dumpChange(2, 1);
                    components[4].dumpLast(1.0, 0);
                }
            }
        }
    }

    private boolean runCollectorMotorsTest(TestResults tr, boolean start, double now){
        
        if (start){
            mLastSampleTime = now;
            mStartTime = now;
        }

        if (now - mLastSampleTime >= .05){
            mLastSampleTime = now;
            tr.process();
        }

        boolean done = false;

        if (mCollector.mPeriodicIO.elbowPosition>Collector.kElbowCollectPosition){
            mCollector.mPeriodicIO.elbowDemand = 0;
            mCollector.mPeriodicIO.rollerDemand = 0;
            tr.dumpResults(true);
            done = true;
        }
        else{
            mCollector.mPeriodicIO.elbowDemand = .5;
            mCollector.mPeriodicIO.rollerDemand = 1.0;
        }

        return done;
    }

    private boolean runCollectorSolenoidTest(TestResults tr, boolean start, double now){

        if (start){
            mLastSampleTime = now;
            mStartTime = now;
        }

        if (now - mLastSampleTime >= .05){
            mLastSampleTime = now;
            tr.process();
        }

        boolean done = false;

        if (now > mStartTime+1.0){
            tr.dumpResults(false);
            done = true;
        }
        else{
            mCollector.mPeriodicIO.solenoidDemand = mCollector.mSolenoidLastState.EXTENDED;
        }

        return done;
    }

    private boolean stowCollector(boolean start, double now){

        if (start){
            mStartTime = now;
        }

        boolean done = false;

        if (mCollector.mPeriodicIO.limit_switch){
            mCollector.mPeriodicIO.elbowDemand = -5.0;
            mCollector.mPeriodicIO.solenoidDemand = mCollector.mSolenoidLastState.RETRACTED;
        }
        else{
            mCollector.mPeriodicIO.elbowDemand = 0;
            done = true;
        }

        return done;
    }

    private boolean runAllCollectorTests(boolean firstTime, double now){
        if (firstTime){
            mCurrentTest = 0;
            mPrevTest = -1;
        }

        boolean newStart = mPrevTest != mCurrentTest;
        boolean allDone = false;

        if (newStart){
            switch (mCurrentTest){
                case 0:
                    mTestResults = new TestResults("Collector Motors");
                    break;
                case 1:
                    mTestResults = new TestResults("Collector Solenoid");
                    break;
                case 2:
                    // no test to run, but not yet done
                    break;
                default:
                    allDone = true;
                    break;
            }
            mPrevTest = mCurrentTest;
        }

        if(!allDone){
            switch (mCurrentTest){
                case 0:
                    if (runCollectorMotorsTest(mTestResults, newStart, now)){
                        mCurrentTest++;
                    }
                    break;
                case 1:
                    if (runCollectorSolenoidTest(mTestResults, newStart, now)){
                        mCurrentTest++;
                    }
                    break;
                case 2:
                    if (stowCollector(newStart, now)){
                        mCurrentTest++;
                    }
                default:
                    allDone = true;
                    break;
            }
        }
        
        return allDone;
    }

    public Collector.SystemState handleSelfTesting(double now) {

        if(mCollector.mStateChanged) {
            mSafeToRun = false;

            if (mCollector.mPeriodicIO.limit_switch == false){
                    mSafeToRun = true;
            } else {
                System.out.println("-------------------------------------------");
                System.out.println("Collector must be stowed to run self test");
                System.out.println("Limit Switch ("+mCollector.mPeriodicIO.limit_switch+") must be false");
            }
        }

        if (mSafeToRun){
            runAllCollectorTests(mCollector.mStateChanged, now);
        } else {
            mCollector.mWantedState = Collector.WantedState.BRING_UP;
        }

        return mCollector.defaultStateTransfer();
    }

}