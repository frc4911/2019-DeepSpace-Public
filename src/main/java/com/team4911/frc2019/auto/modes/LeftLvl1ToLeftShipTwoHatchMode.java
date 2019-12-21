package com.team4911.frc2019.auto.modes;

import java.util.Arrays;

import com.team254.lib.autos.AutoModeBase;
import com.team254.lib.autos.AutoModeEndedException;
import com.team254.lib.autos.actions.ParallelAction;
import com.team4911.frc2019.auto.actions.CollectHatch;
import com.team4911.frc2019.auto.actions.DriveTrajectory;
import com.team4911.frc2019.auto.actions.HomeElevator;
import com.team4911.frc2019.auto.actions.ShootHatch;
import com.team4911.frc2019.paths.TrajectoryGenerator;

import edu.wpi.first.wpilibj.Timer;

public class LeftLvl1ToLeftShipTwoHatchMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        double start = Timer.getFPGATimestamp();
        runAction(new HomeElevator());
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().lvl1ToShipBayNear.left, true));
        runAction(new ShootHatch(true));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().shipBayNearToStation.left, false));
        runAction(new ParallelAction(Arrays.asList(new ShootHatch(false), 
                  new CollectHatch(true), 
                  new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().shipBayNearToStation2.left, false))));
        runAction(new CollectHatch(false));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().stationToShipBayMid.left, false));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().stationToShipBayMid2.left, false));
        runAction(new ShootHatch(true));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().shipBayMidToStation.left, false));
        double stop = Timer.getFPGATimestamp();
        System.out.println("auto dur: " + (stop - start));
    }

}