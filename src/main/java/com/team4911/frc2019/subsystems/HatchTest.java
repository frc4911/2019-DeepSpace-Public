package com.team4911.frc2019.subsystems;

import com.team4911.frc2019.subsystems.HatchIntake.BeakSolenoidState;
import com.team4911.frc2019.subsystems.HatchIntake.EjectSolenoidState;
import com.team4911.frc2019.utils.TestValueTracker;

public class HatchTest {
    
    private HatchIntake mHatch;
    private int mCurrentTest = 0;
    private int mPrevTest = -1;
    private double mPrepTimeout = 0;
    private double mTestTimeout = 0;
    private TestResults mTestResults = null;

    public HatchTest (){
        this.mHatch = HatchIntake.getInstance();
    }

    public HatchIntake.SystemState handleBringingUp(){
        return mHatch.defaultStateTransfer();
    }

    public void setBeak(HatchIntake.BeakSolenoidState state){
        if (mHatch.mSystemState == HatchIntake.SystemState.BRINGING_UP){
            mHatch.mPeriodicIO.beak_demand = state;
        }
    }

    public void setPiston(HatchIntake.EjectSolenoidState state){
        if (mHatch.mSystemState == HatchIntake.SystemState.BRINGING_UP){
            mHatch.mPeriodicIO.eject_demand = state;
        }
    }

    private class TestResults{

        String description;

        public TestResults (String description){
            this.description = description;
        }
        TestValueTracker[] components = {new TestValueTracker("solenoid psi")};

        public void process(){
            components[0].process(mHatch.mPeriodicIO.pneumaticPressure);
        }

        public void dumpResults(){
            System.out.println("---------------------------------------");
            System.out.println(description);

            components[0].dumpChange(2.0, 1.0);
        }
    }

    private boolean runAllHatchSelfTests(boolean firstTime, double now){

        if (firstTime){
            mCurrentTest = 0;
            mPrevTest = -1;

            // beak must be closed and pistons retracted
            mPrepTimeout = now;
            if (mHatch.mPeriodicIO.beak_demand == BeakSolenoidState.OPEN || 
                mHatch.mPeriodicIO.eject_demand == EjectSolenoidState.RETRACT){
                mPrepTimeout += 1; // 1 second timeout
            }
            mHatch.mPeriodicIO.beak_demand = BeakSolenoidState.CLOSE;
            mHatch.mPeriodicIO.eject_demand = EjectSolenoidState.RETRACT;
        }

        if (now < mPrepTimeout){
            return false;
        }

        boolean newStart = mPrevTest != mCurrentTest;
        boolean allDone = false;

        if (newStart){
            switch (mCurrentTest){
                case 0:
                    mTestResults = new TestResults("Beak open test");
                    mHatch.mPeriodicIO.beak_demand = BeakSolenoidState.OPEN;
                    mTestTimeout = now + 1;
                    break;
                case 1:
                    mTestResults = new TestResults("Beak close test");
                    mHatch.mPeriodicIO.beak_demand = BeakSolenoidState.CLOSE;
                    mTestTimeout = now + 1;
                    break;
                case 2:
                    mTestResults = new TestResults("Pistons eject test");
                    mHatch.mPeriodicIO.eject_demand = EjectSolenoidState.EJECT;
                    mTestTimeout = now + 1;
                    break;
                case 3:
                    mTestResults = new TestResults("Pistons retract test");
                    mHatch.mPeriodicIO.eject_demand = EjectSolenoidState.RETRACT;
                    mTestTimeout = now + 1;
                    break;
                default:
                    allDone = true;
                    break;
            }
            mPrevTest = mCurrentTest;
        }

        if (!allDone && (now > mTestTimeout)){
            mTestResults.dumpResults();
            mCurrentTest++;
        }
        else {
            mTestResults.process();
        }
        return allDone;
    }

    public HatchIntake.SystemState handleSelfTesting(double now){

        if (runAllHatchSelfTests(mHatch.mStateChanged, now)){
            mHatch.setWantedState(HatchIntake.WantedState.BRING_UP);
        }
        return mHatch.defaultStateTransfer();
    }
}