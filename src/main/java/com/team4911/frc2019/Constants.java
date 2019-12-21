package com.team4911.frc2019;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import com.team4911.frc2019.utils.Annot4911;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.01;
    public static final int kTimeOutMs = 30;
    public static final int kMagEncFramePer = 10;

    public static final String kSilverName = "Silver";
    public static final String kChromeName = "Chrome";
    public static final String kWeebleName = "Weeble";
    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 21.5;
    public static final double kDriveWheelDiameterInches = 4.0;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 65.0; // 60.0 // kg TODO tune
    public static final double kRobotAngularInertia = 10.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.055;  // V
    public static final double kDriveKv = 0.135;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2

    // Geometry: 31x27 frame + 2.75 bumpers
    public static final double kCenterToFrontBumperDistance = (31 + (2 * 2.75)) / 2.0;
    public static final double kCenterToRearBumperDistance = (31 + (2 * 2.75)) / 2.0;
    public static final double kCenterToSideBumperDistance = (27 + (2 * 2.75)) / 2.0;

    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0;  // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0;  // inches per second

    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveLowGearVelocityKp = 0.9;
    public static final double kDriveLowGearVelocityKi = 0.0;
    public static final double kDriveLowGearVelocityKd = 10.0;
    public static final double kDriveLowGearVelocityKf = 0.0;
    public static final int kDriveLowGearVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    // PID gains for elevator position loop
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kElevatorPositionKp = 0.1;
    public static final double kElevatorPositionKi = 0.0;
    public static final double kElevatorPositionKd = 0.0;
    public static final double kElevatorPositionKf = 0.0;
    public static final int kElevatorPositionIZone = 0;

    // PID gains for elvator motion magic loop
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kElevatorMotionMagicKp = 0.5115; // 0.008;
    public static final double kElevatorMotionMagicKi = 0.0;
    public static final double kElevatorMotionMagicKd = 0.0;
    public static final double kElevatorMotionMagicKf = 0.164; // 0.327;
    public static final int kElevatorMotionMagicIZone = 0;
    public static final int kElevatorCruiseVelocity = 6252; // 3126
    public static final int kElevatorAccelerationUp = 6252 * 3; // 3126
    public static final int kElevatorAccelerationDown = 6252; // 3126
    public static final double kElevatorVoltageRampRate = 0.05;

    // Drive Train
    @Annot4911(description = "CAN_PDP 0 - drive left master talon")
    public static final int kDriveLeftMasterId = 0;
    @Annot4911(description = "CAN_PDP 1 - drive left slave A talon")
    public static final int kDriveLeftSlaveAId = 1;
    @Annot4911(description = "CAN_PDP 2 - drive left slave B talon")
    public static final int kDriveLeftSlaveBId = 2;
    @Annot4911(description = "CAN_PDP 15 - drive right master talon")
    public static final int kDriveRightMasterId = 15;
    @Annot4911(description = "CAN_PDP 14 - drive right slave A talon")
    public static final int kDriveRightSlaveAId = 14;
    @Annot4911(description = "CAN_PDP 13 - drive right slave B talon")
    public static final int kDriveRightSlaveBId = 13;
    @Annot4911(description = "PCM 0 - drive gear shift solenoid")
    public static final int kDriveShifterId = 0;
    @Annot4911(description = "Talon 0 Breakout - drive left encoder")
    public static final int kAnnot0Id = 0;
    @Annot4911(description = "Talon 15 Breakout - drive right encoder")
    public static final int kAnnot1Id = 0;
    @Annot4911(description = "Talon 1 Breakout - Pigeon IMU")
    public static final int kAnnot2Id = 0;
    @Annot4911(description = "I2C - line sensor arrays + Arduino")
    public static final int kAnnot3Id = 0;

    // Elevator
    @Annot4911(description = "CAN_PDP 3 - elevator master talon")
    public static final int kElevatorMasterId = 3;
    @Annot4911(description = "CAN_PDP 12 - elevator slave talon")
    public static final int kElevatorSlaveId = 12;
    @Annot4911(description = "Talon 4 Breakout - elevator top limit switch")
    public static final int kAnnot12Id = 0;
    @Annot4911(description = "Talon 4 Breakout - elevator bottom limit switch")
    public static final int kAnnot13Id = 0;
    @Annot4911(description = "Analog 0 - elevator string potentiometer")
    public static final int kElevatorPotId = 0;
    @Annot4911(description = "Talon 4 breakout - elevator encoder")
    public static final int kAnnot4Id = 0;

    // Collector
    @Annot4911(description = "CAN_PDP 10 - collector spin talon")
    public static final int kCollectorSpinId = 10;
    @Annot4911(description = "CAN_PDP 6 - collector elbow talon")
    public static final int kCollectorElbowId = 6;
    @Annot4911(description = "Talon 6 Breakout - collector elbow encoder")
    public static final int kCollectorElbowEncoderId = 0; 
    @Annot4911(description = "Talon 6 Breakout - collector elbow collapsed limit switch")
    public static final int kAnnot7Id = 0;
    @Annot4911(description = "Talon 6 Breakout - collector elbow extended limit switch")
    public static final int kAnnot8Id = 0;
    @Annot4911(description = "PCM 1 - collector deploy solenoid")
    public static final int kCollectorDeployId = 1; 
    @Annot4911(description = "kP for the collector Position PID")
    public static final double kCollectorPID_P = 0.005; // supposing that the output is .5 when error is 100
    @Annot4911(description = "kI for the collector Position PID")
    public static final double kCollectorPID_I = 0;
    @Annot4911(description = "kD for the collector Position PID")
    public static final double kCollectorPID_D = 0;
    
    // Shooter
    @Annot4911(description = "CAN_PDP 11 - shooter talon")
    public static final int kShooterId = 11;
    @Annot4911(description = "Talon 11 Breakout - shooter encoder")
    public static final int kAnnot9Id = 0;
    @Annot4911(description = "DIO 0 - shooter beam break limit switch")
    public static final int kShooterBeamBreakId = 0;

    // Climber
    @Annot4911(description = "PCM 2 - climber deploy solenoid")
    public static final int kClimberDeployId = 2;
    @Annot4911(description = "PCM 3 - climber retract solenoid")
    public static final int kClimberRetractId = 3;
    // @Annot4911(description = "CAN_PDP 3 - climber curl master talon")
    // public static final int kClimberCurlMasterId = 3;
    // @Annot4911(description = "CAN_PDP 12 - climber curl slave talon")
    // public static final int kClimberCurlSlaveId = 12;
    // @Annot4911(description = "PCM 2 - climber deploy double solenoid A")
    // public static final int kClimberDeployAId = 2;
    // @Annot4911(description = "PCM 3 - climber deploy double solenoid B")
    // public static final int kClimberDeployBId = 3;
    // @Annot4911(description = "PCM 4 - climber clamp double solenoid A")
    // public static final int kClimberClampAId = 4;
    // @Annot4911(description = "PCM 5 - climber clamp double solenoid B")
    // public static final int kClimberClampBId = 5;
    // @Annot4911(description = "Talon 3 Breakout - climber lifted limit switch")
    // public static final int kAnnot10Id = 0;
    // @Annot4911(description = "Talon 3 Breakout - climber home limit switch")
    // public static final int kAnnot11Id = 0;

    // Hatch Beak
    @Annot4911(description = "PCM 6 - open beak solenoid")
    public static final int kOpenBeakId = 6;
    @Annot4911(description = "PCM 7 - hatch deploy solenoid")
    public static final int kHatchDeployId = 7;
    @Annot4911(description = "I2C - safe to deploy hatch lidar")
    public static final int kAnnot14Id = 0;
    @Annot4911(description = "Analog 1 - pneumatic pressure sensor")
    public static final int kPressureId = 1;


    // LED/Canifier
    public static final int kCanifierId = 0;

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/179YszqnEWPWInuHUrYJnYL48LUL7LUhZrnvmNu1kujE/edit#gid=0

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors

    // Control Board
    public static final int kMainThrottleJoystickPort = 1;
    public static final int kMainTurnJoystickPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final double kJoystickThreshold = 0.5;
    public static final double kJoystickJogThreshold = 0.4;

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
