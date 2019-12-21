package com.team4911.frc2019.subsystems;

import com.team254.lib.util.RobotName;
import com.team4911.frc2019.Constants;
import com.team4911.frc2019.utils.TestValueTracker;

public class DriveTest {

    private Drive mDrive = Drive.getInstance();

    private TestResults mTestResults;
    private double mLastSampleTime;
    private double mStartTime;
    private int mCurrentTest = 0;
    private int mPrevTest = -1;

    private void stopAllMotors(){
        mDrive.mPeriodicIO.left_demand = 0;
        mDrive.mPeriodicIO.right_demand = 0;
    }

    private class TestResults{

        TestValueTracker[] components = {new TestValueTracker("Left Master talon amps"),
                                    new TestValueTracker("Left SlaveA talon amps"),
                                    new TestValueTracker("Left SlaveB talon amps"),
                                    new TestValueTracker("Right Master talon amps"),
                                    new TestValueTracker("Right SlaveA talon amps"),
                                    new TestValueTracker("Right SlaveB talon amps"),
                                    new TestValueTracker("Left encoder counts"),
                                    new TestValueTracker("Right encoder counts"),
                                    // new TestValueTracker("Shifter solenoid psi"),
                                };

        public void process(){
            components[0].process(mDrive.mPeriodicIO.leftMaster_current);
            components[1].process(mDrive.mPeriodicIO.leftSlaveA_current);
            components[2].process(mDrive.mPeriodicIO.leftSlaveB_current);
            components[3].process(mDrive.mPeriodicIO.rightMaster_current);
            components[4].process(mDrive.mPeriodicIO.rightSlaveA_current);
            components[5].process(mDrive.mPeriodicIO.rightSlaveB_current);
            components[6].process(mDrive.mPeriodicIO.left_position_ticks);
            components[7].process(mDrive.mPeriodicIO.right_position_ticks);
            // components[8].process(mHatch.mPeriodicIO.pressureSensor);
        }

        public void dumpResults(boolean shifter){

            System.out.println("---------------------------------------");
            System.out.println("Drive Test");
            if (RobotName.name.equals(Constants.kSilverName)){
                if (shifter){
                    System.out.println("High Gear");
                    components[0].dumpMax(9.0, 1);
                    components[0].dumpAve(2.0, 1);
                    components[1].dumpMax(9.0, 1);
                    components[1].dumpAve(2.0, 1);
                    components[2].dumpMax(9.0, 1);
                    components[2].dumpAve(2.0, 1);    
        
                    components[3].dumpMax(9.0, 1);
                    components[3].dumpAve(2.0, 1);
                    components[4].dumpMax(9.0, 1);
                    components[4].dumpAve(2.0, 1);
                    components[5].dumpMax(9.0, 1);
                    components[5].dumpAve(2.0, 1);    
        
                    // encoders
                    components[6].dumpMax(75000, 10000);
                    components[6].dumpLast(0, 10000);
                    
                    components[7].dumpMax(75000, 10000);
                    components[7].dumpLast(0, 10000);
        
                    // gear shift
                    // components[7].dumpChange(1, .5);
        
                }
                else{
                    System.out.println("Low Gear");
                    components[0].dumpMax(7.0, 1);
                    components[0].dumpAve(2.0, 1);
                    components[1].dumpMax(7.0, 1);
                    components[1].dumpAve(2.0, 1);
                    components[2].dumpMax(7.0, 1);
                    components[2].dumpAve(2.0, 1);    

                    components[3].dumpMax(7.0, 1);
                    components[3].dumpAve(2.0, 1);
                    components[4].dumpMax(7.0, 1);
                    components[4].dumpAve(2.0, 1);
                    components[5].dumpMax(7.0, 1);
                    components[5].dumpAve(2.0, 1);    
        
                    // encoders
                    components[6].dumpMax(75000, 10000);
                    components[6].dumpLast(0, 10000);
                    
                    components[7].dumpMax(75000, 10000);
                    components[7].dumpLast(0, 10000);
        
                    // gear shift
                    // components[7].dumpChange(1, .5);
        
                }
            }
            else {
                // Chrome
                if (shifter){
                    System.out.println("High Gear");
                    components[0].dumpMax(3.5, 1);
                    components[0].dumpAve(1.0, 1);
                    components[1].dumpMax(7.5, 1);
                    components[1].dumpAve(2.5, 1);
                    components[2].dumpMax(3.0, 1);
                    components[2].dumpAve(1.0, 1);

                    components[3].dumpMax(5.0, 1);
                    components[3].dumpAve(1.5, 1);
                    components[4].dumpMax(4.5, 1);
                    components[4].dumpAve(1.5, 1);
                    components[5].dumpMax(7.0, 1);
                    components[5].dumpAve(2.5, 1);

                    // encoders
                    components[6].dumpMax(276000, 10000);
                    components[6].dumpLast(-1000, 10000);

                    components[7].dumpMax(264000, 10000);
                    components[7].dumpLast(3000, 10000);

                    // gear shift
                    components[7].dumpChange(1, .5);

                }
                else{
                    System.out.println("Low Gear");
                    components[0].dumpMax(2.0, 1);
                    components[0].dumpAve(1.0, 1);
                    components[1].dumpMax(5.5, 1);
                    components[1].dumpAve(2.0, 1);
                    components[2].dumpMax(2.0, 1);
                    components[2].dumpAve(1.0, 1);

                    components[3].dumpMax(3.0, 1);
                    components[3].dumpAve(1.0, 1);
                    components[4].dumpMax(3.5, 1);
                    components[4].dumpAve(1.0, 1);
                    components[5].dumpMax(4.5, 1);
                    components[5].dumpAve(1.5, 1);

                    // encoders
                    components[6].dumpMax(132000, 10000);
                    components[6].dumpLast(-1000, 10000);

                    components[7].dumpMax(132000, 10000);
                    components[7].dumpLast(3800, 10000);

                    // gear shift
                    components[7].dumpChange(1, .5);
                }
            }
        }
    }
    
    private boolean runDriveTest(TestResults tr, boolean start, double now){

        // double now = Timer.getFPGATimestamp();

        if (start){
            mLastSampleTime = now;
            mStartTime = now;
        }

        if (now - mLastSampleTime >= .1){
            mLastSampleTime = now;
            tr.process();
        }

        // set speed based on time
        double speed = (now-mStartTime)/3.0;  // 3.0 means 3.0 x 4 = 12.0 second test duration

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
            stopAllMotors();
            tr.dumpResults(mDrive.isHighGear());
        }
        else {
            mDrive.mPeriodicIO.left_demand = mDrive.mPeriodicIO.right_demand = speed;
        }

        return speed >= 4.0;
    }

    double mGearShiftTimeout = 0;

    private boolean runAllDriveTests(boolean firstTime, double now){

        if (firstTime){
            mCurrentTest = 0;
            mPrevTest = -1;
            // must be in low gear to start
            mGearShiftTimeout = now;
            if (!mDrive.isHighGear()){
                mGearShiftTimeout += 1; // 1 second timeout
                mDrive.setHighGear(false);
            }
        }

        if (now < mGearShiftTimeout){
            return false;
        }

        boolean newStart = mPrevTest != mCurrentTest;
        boolean allDone = false;

        if (newStart){
            switch (mCurrentTest){
                case 0:
                    mDrive.setHighGear(true);
                    mTestResults = new TestResults();
                    break;
                case 1:
                    mDrive.setHighGear(false);
                    mTestResults = new TestResults();
                    break;
                default:
                    allDone = true;
                    break;
            }
            mPrevTest = mCurrentTest;
        }

        if (!allDone){
            if (runDriveTest(mTestResults, newStart, now)){
                mCurrentTest++;
            }
        }
        
        return allDone;
    }

    public Drive.SystemState handleSelfTesting(double now){
        if (runAllDriveTests(mDrive.mStateChanged, now)){
            mDrive.mWantedState = Drive.WantedState.BRING_UP;
        }
        return mDrive.defaultStateTransfer();
    }

    public Drive.SystemState handleBringUp(){
        if (mDrive.mStateChanged){
            stopAllMotors();
        }
        return mDrive.defaultStateTransfer();
    }

    public void setSpeed (double speed){
        mDrive.mPeriodicIO.left_demand = mDrive.mPeriodicIO.right_demand = speed;
    }
}