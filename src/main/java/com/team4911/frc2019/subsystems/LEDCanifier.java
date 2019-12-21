package com.team4911.frc2019.subsystems;

import com.ctre.phoenix.CANifier;
// import com.ctre.phoenix.CANifierStatusFrame;
import com.team254.lib.subsystems.Subsystem;
import com.team4911.frc2019.Constants;

/**
 * Provides an LED controller using a CTR Canifier.
 */
public class LEDCanifier extends Subsystem {
    private static LEDCanifier mInstance;
    private CANifier mCanifier;
    private PeriodicOutputs mPeriodicOutputs;
    private boolean mOutputsChanged = true;

    private LEDCanifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.configFactoryDefault(Constants.kLongCANTimeoutMs);
//        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, Constants.kLongCANTimeoutMs);
//        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        mPeriodicOutputs = new PeriodicOutputs();

        // Force a first update.
        mOutputsChanged = true;
    }

    public synchronized static LEDCanifier getInstance() {
        if (mInstance == null) {
            mInstance = new LEDCanifier();
        }
        return mInstance;
    }

    public int getDeviceId() {
        return mCanifier.getDeviceID();
    }

    public synchronized void setLEDColor(double red, double green, double blue) {
        if (red != mPeriodicOutputs.red || green != mPeriodicOutputs.green || blue != mPeriodicOutputs.blue) {
            mPeriodicOutputs.red = red;
            mPeriodicOutputs.green = green;
            mPeriodicOutputs.blue = blue;
            mOutputsChanged = true;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // A: Blue
        // B: Green
        // C: Red
        if (mOutputsChanged) {
            mCanifier.setLEDOutput(mPeriodicOutputs.green, CANifier.LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(mPeriodicOutputs.red, CANifier.LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(mPeriodicOutputs.blue, CANifier.LEDChannel.LEDChannelC);
            mOutputsChanged = false;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
        mPeriodicOutputs = new PeriodicOutputs();
        mOutputsChanged = true;
        writePeriodicOutputs();
    }

    @Override
    public void zeroSensors() {
    }

    private static class PeriodicOutputs {
        public double red;
        public double green;
        public double blue;
    }
}
