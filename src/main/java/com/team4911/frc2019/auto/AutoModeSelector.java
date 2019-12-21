package com.team4911.frc2019.auto;

import com.team254.lib.autos.AutoModeBase;
import com.team4911.frc2019.auto.creators.AutoModeCreator;
import com.team4911.frc2019.auto.creators.DriveBackwardsModeCreator;
import com.team4911.frc2019.auto.creators.RocketTwoHatchNearModeCreator;
import com.team4911.frc2019.auto.creators.ShipTwoHatchModeCreator;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    public enum StartingPosition {
        LEFT,
        CENTER,
        RIGHT,
    };

    public enum PlatformPosition {
        LEVEL_ONE,
        LEVEL_TWO
    };

    enum DesiredMode {
        DO_NOTHING,
        ROCKET_HATCH_NEAR,
        DRIVE_BACKWARDS,
        SHIP_HATCH,
        SHIP_CARGO,
        ADVANCED, // This uses 4 additional sendable choosers to pick one for each field state combo
    };

    private DesiredMode mCachedDesiredMode = null;
    private PlatformPosition mCachedPlatformPosition = null;
    private StartingPosition mCachedStartingPosition = null;

    private Optional<AutoModeCreator> mCreator = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<PlatformPosition> mPlatformPositionChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Rocket Hatch Near", DesiredMode.ROCKET_HATCH_NEAR);
        mModeChooser.addOption("Drive Backwards", DesiredMode.DRIVE_BACKWARDS);
        mModeChooser.addOption("Ship Hatch", DesiredMode.SHIP_HATCH);
        mModeChooser.addOption("Ship Cargo", DesiredMode.SHIP_CARGO);
        SmartDashboard.putData("Auto mode", mModeChooser);

        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Right", StartingPosition.RIGHT);
        mStartPositionChooser.addOption("Center", StartingPosition.CENTER);
        mStartPositionChooser.addOption("Left", StartingPosition.LEFT);
        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mPlatformPositionChooser = new SendableChooser<>();
        mPlatformPositionChooser.setDefaultOption("Level 1", PlatformPosition.LEVEL_ONE);
        mPlatformPositionChooser.addOption("Level 2", PlatformPosition.LEVEL_TWO);
        SmartDashboard.putData("Starting Level", mPlatformPositionChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        PlatformPosition platformPosition = mPlatformPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition || platformPosition != mCachedPlatformPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name() + ", starting position->" + startingPosition.name() + ", platform position->" + platformPosition.name());
            mCreator = getCreatorForParams(desiredMode, startingPosition, platformPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
        mCachedPlatformPosition = platformPosition;
    }

    private Optional<AutoModeCreator> getCreatorForParams(DesiredMode mode, StartingPosition position, PlatformPosition level) {
       switch (mode) {
           case ROCKET_HATCH_NEAR:
               return Optional.of(new RocketTwoHatchNearModeCreator(position, level));
           case DRIVE_BACKWARDS:
               return Optional.of(new DriveBackwardsModeCreator());
           case SHIP_HATCH:
               return Optional.of(new ShipTwoHatchModeCreator(position, level));
           case SHIP_CARGO:
               return Optional.empty();
           default:
               break;
       }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mCreator = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
        SmartDashboard.putString("PlatformPositionSelected", mCachedPlatformPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mCreator.isPresent()) {
            return Optional.empty();
        }

        return Optional.of(mCreator.get().getAutoMode());
    }
}
