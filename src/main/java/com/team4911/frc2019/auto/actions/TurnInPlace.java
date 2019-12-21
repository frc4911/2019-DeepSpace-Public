package com.team4911.frc2019.auto.actions;

import com.team4911.frc2019.subsystems.Drive;
import com.team254.lib.autos.actions.Action;
import com.team254.lib.util.DriveSignal;

public class TurnInPlace implements Action {
    private static final double kMaxSpeed = 0.72;  // fastest we want to go
    private static double kMinSpeed = 0.24;  // still keep moving ever so slowly
    private static double lastError = Double.MAX_VALUE;
    private double crossTarget;
    private int count = 0;

    private static final Drive mDrive = Drive.getInstance();

    private double angle = 0.0;

    /**
     * Turns in place to the specified angle.  This is field relative wrt.
     * where the robot was initially was started (i.e. where the gyro lasr was zero'd)  
     * Angles are  [-180, 180] where positive angle is counter clockwise.
     *
     * @param angle to turn to.
     */
    public TurnInPlace(double angle) { this.angle = normalizeAngle(angle); }

    @Override
    public boolean isFinished() {
        return periodic();
    }

    @Override
    public void update() {
        //periodic();
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
    }

    @Override
    public void start() {
        count = 0;
        lastError = normalizeAngle(angle - mDrive.getHeading().getDegrees());
        crossTarget = 0;
        // kMinSpeed = 0.24;
    }

    public boolean periodic() {
        // Calculate error with directional bias.  If negative turn clockwise, otherwise counter clockwise.
        double currentAngle = mDrive.getHeading().getDegrees();
        double error = normalizeAngle(angle - currentAngle);

        //cross target twice to be done
        if (Math.signum(lastError) != Math.signum(error)) {
            crossTarget++;
            lastError = error;
            // kMinSpeed-= .01;
        }

        // To maximize rotation speed, use sin(radians(speed/2)) with clamping

        double newSpeed = Math.signum(error) * kMinSpeed + Math.sin(Math.toRadians(error/2)) / (kMaxSpeed - kMinSpeed);
        // double newSpeed = Math.sin(Math.toRadians(error/4.0)) / (kMaxSpeed - kMinSpeed);

        // // clamp top speed
        // if (newSpeed > 0){
        //     newSpeed = Math.min(kMaxSpeed,newSpeed);
        // }
        // else {
        //     newSpeed = Math.max(-kMaxSpeed,newSpeed);
        // }

        // // clamp low speed
        // if (newSpeed>0){
        //     newSpeed = Math.max(kMinSpeed,newSpeed);
        // }
        // else{
        //     newSpeed = Math.min(-kMinSpeed,newSpeed);
        // }

        System.out.println("error=" + error + 
            " speed=" + newSpeed +
            " target angle=" + angle + 
            " current angle=" + currentAngle +
            " count="+count);

        if (crossTarget < 3){//Math.abs(error) > kDeadband ) {
            mDrive.setOpenLoop(new DriveSignal(-newSpeed , newSpeed ));
            return false;
        }
        mDrive.setOpenLoop(new DriveSignal(0 , 0));
        return count-- < 0;//true;
    }

    /**
     * Maps angle onto -180 to  180 degrees
     * 
     * @param angle to normalize
     * @return normalized angle
     */
    private double normalizeAngle(double angle) {
        return angle + ((angle > 180) ? -360 : (angle < -180) ? 360 : 0);
    }
}
