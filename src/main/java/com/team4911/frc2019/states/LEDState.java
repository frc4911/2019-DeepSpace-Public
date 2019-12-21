package com.team4911.frc2019.states;

public class LEDState {
    public static final LEDState kOff = new LEDState(0.0, 0.0, 0.0);

//    public static final LEDState kIntakeOpenLoop = new LEDState(0.0, 0.0, 0.0);

    // Collector  State
    //  Deployed = Blue
    //  Has Cargo = Orange
    public static final LEDState kCollectorDeployed = new LEDState(1.0, 0.0, 0.0);
    public static final LEDState kHasCargo = new LEDState(0.0, 0.25, 1.0);

    // Drive (purple)
    public static final LEDState kDrive = new LEDState(0.5, 0.0, 0.5);

    // Intake BeakDown
    public static final LEDState kBeakDown = new LEDState(0.0, 1.0, 0.0);

    public static final LEDState kFault = new LEDState(0.0, 0.0, 1.0);

    public static final LEDState kClimbing = new LEDState(0.0, 0.3, 1.0);

    // Yellow
    public static final LEDState kAutoStart = new LEDState(0.0, 1.0, 1.0);

    public LEDState() {
    }

    public LEDState(double b, double g, double r) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LEDState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public double blue;
    public double green;
    public double red;
}
