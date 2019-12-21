package com.team4911.frc2019.states;

public interface TimedLEDState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        public static BlinkingLEDState kHasCargo = new BlinkingLEDState(
                LEDState.kHasCargo,
                LEDState.kOff,
                0.5);

        public static BlinkingLEDState kCollectorDeployed = new BlinkingLEDState(
                LEDState.kCollectorDeployed,
                LEDState.kOff,
                0.5);

        public static BlinkingLEDState kBeakDown = new BlinkingLEDState(
                LEDState.kBeakDown,
                LEDState.kOff,
                0.5);

        public static BlinkingLEDState kDrive = new BlinkingLEDState(
                LEDState.kDrive,
                LEDState.kOff,
                0.5);

        LEDState mStateOne = new LEDState(0.0, 0.0, 0.0);
        LEDState mStateTwo = new LEDState(0.0, 0.0, 0.0);
        double mDuration;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if ((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }
        }
    }

    class StaticLEDState implements TimedLEDState {
        public static StaticLEDState kStaticOff = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kCollectorDeployed = new StaticLEDState(LEDState.kCollectorDeployed);
        public static StaticLEDState kBeakDown = new StaticLEDState(LEDState.kBeakDown);
        public static StaticLEDState kHasCargo = new StaticLEDState(LEDState.kHasCargo);
        public static StaticLEDState kDrive = new StaticLEDState(LEDState.kDrive);
        public static StaticLEDState kAutoStart = new StaticLEDState(LEDState.kAutoStart);

        LEDState mStaticState = new LEDState(0.0, 0.0, 0.0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }
}
