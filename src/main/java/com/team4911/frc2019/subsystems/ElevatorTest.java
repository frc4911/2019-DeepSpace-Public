package com.team4911.frc2019.subsystems;

import com.team254.lib.util.RobotName;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.utils.TestValueTracker;

public class ElevatorTest {

    private Elevator mElevator;
    private TestResults mTestResults = null;
    private double mLastSampleTime;
    private double mStartTime;
    private int mCurrentTest = 0;
    private int mPrevTest = -1;
    private boolean mSafeToRun = false;
    
    public ElevatorTest() {
        this.mElevator = Elevator.getInstance();
    }

    public Elevator.SystemState handleBringingUp() {
        return mElevator.defaultStateTransfer();
    }

    public void setElevatorMove(double speed){
        if(mElevator.mSystemState == Elevator.SystemState.BRINGING_UP) {
            mElevator.mPeriodicIO.demand = speed;
            mElevator.mPeriodicIO.feedforward = 0.0;
        }
    }

    private class TestResults{

        TestValueTracker[] components = {new TestValueTracker("Master amps"),
                                        new TestValueTracker("Slave amps"),
                                        new TestValueTracker("Potentiometer detents"),
                                        new TestValueTracker("encoder counts"),
                                        new TestValueTracker("Limit switch value"),
                                      };

        public void process(){
            components[0].process(mElevator.mPeriodicIO.master_current);
            components[1].process(mElevator.mPeriodicIO.slave_current);
            components[2].process(mElevator.mPeriodicIO.string_pot_value);
            components[3].process(mElevator.mPeriodicIO.position_ticks);
            components[4].process(mElevator.mPeriodicIO.bottom_limit);
        }

        public void dumpResults(){

            System.out.println("---------------------------------------");
            System.out.println("Elevator Test");
            if (RobotName.name.equals(Constants.kSilverName)){
                components[0].dumpMax(4.0, 1);
                components[0].dumpAve(4.0, 1);
                components[1].dumpMax(4.0, 1);
                components[1].dumpAve(4.0, 1);
                // pot
                components[2].dumpMax(575, 30);
                
                // encoder
                components[3].dumpMax(25300, 900);
                
                // limit switch
                components[4].dumpLast(0, 0);
                }
                else {
                // Chrome
                components[0].dumpMax(9.0, 1);
                components[0].dumpAve(8.0, 1);
                components[1].dumpMax(9.0, 1);
                components[1].dumpAve(7.5, 1);
                // pot
                components[2].dumpMax(1195, 100);
                
                // encoder
                components[3].dumpMax(49000, 1000);
                
                // limit switch
                components[4].dumpLast(0, 0);
            }
        }
    }
    
    private boolean runElevatorTest(TestResults tr, boolean start, double now){

        boolean allDone = false;

        if (start){
            mLastSampleTime = now;
            mStartTime = now;
        }

        // run for 1 second
        double duration = now-mStartTime;
        double speed = 0;

        if (duration < 2.0) {
            if (now - mLastSampleTime >= .1){
                mLastSampleTime = now;
                tr.process();
            }
            speed = .75;
        }
        else if (duration > 2.0) {
            tr.dumpResults();
            allDone = true;
            speed = 0;
        }

        mElevator.mPeriodicIO.demand = speed;

        return allDone;
    }

    private boolean runAllElevatorTests(boolean firstTime, double now){

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

        if (!allDone && runElevatorTest(mTestResults, newStart, now)){
            mCurrentTest++;
        }
        return allDone;
    }

    public Elevator.SystemState handleSelfTesting(double now){
        if(mElevator.mStateChanged) {
            mSafeToRun = false;

        if (mElevator.mPeriodicIO.bottom_limit == true){
                    mSafeToRun = true;
            }
            else {
                System.out.println("-------------------------------------------");
                System.out.println("Elevator must be at bottom to run self test");
                System.out.println("Limit Switch ("+mElevator.mPeriodicIO.bottom_limit+") must be true");
            }
        }

        if (mSafeToRun){
            if (runAllElevatorTests(mElevator.mStateChanged, now)){
                mElevator.mWantedState = Elevator.WantedState.BRING_UP;
            }
        } else {
            mElevator.mWantedState = Elevator.WantedState.BRING_UP;
        }
        
        return mElevator.defaultStateTransfer();
    }
}