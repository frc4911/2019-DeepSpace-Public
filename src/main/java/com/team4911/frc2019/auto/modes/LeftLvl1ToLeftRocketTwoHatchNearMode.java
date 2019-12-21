package com.team4911.frc2019.auto.modes;

import com.team254.lib.autos.AutoModeBase;
import com.team254.lib.autos.AutoModeEndedException;
import com.team254.lib.autos.actions.ParallelAction;
import com.team254.lib.autos.actions.WaitAction;
import com.team4911.frc2019.auto.actions.CollectHatch;
import com.team4911.frc2019.auto.actions.DriveTrajectory;
import com.team4911.frc2019.auto.actions.HomeElevator;
import com.team4911.frc2019.auto.actions.MoveElevatorToPreset;
import com.team4911.frc2019.auto.actions.ShootHatch;
import com.team4911.frc2019.paths.TrajectoryGenerator;
import com.team4911.frc2019.auto.actions.TurnInPlace;
import com.team4911.frc2019.subsystems.Elevator;

public class LeftLvl1ToLeftRocketTwoHatchNearMode extends AutoModeBase {

	@Override
    protected void routine() throws AutoModeEndedException {
        // runAction(new HomeElevator());
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().leftLvl1ToLeftRocket, true));
        // runAction(new ShootHatch(true));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().leftRocketToLoadStation, false));
        // runAction(new ShootHatch(false));
        // runAction(new CollectHatch(true));
        // runAction(new TurnInPlace(180.0));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().leftRocketToLoadStation2, false));
        // runAction(new CollectHatch(false));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().loadStationToLeftRocket, false));
        // runAction(new TurnInPlace(180.0));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().loadStationToLeftRocket2, false));
        // runAction(new MoveElevatorToPreset(Elevator.PositionPresets.HATCH_LVL_2));
        // runAction(new ShootHatch(true));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().leftRocketToLoadStation, false));

        // runAction(new ParallelAction(ArrayList.asList(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().loadStationToLeftRocket2, false),
        //           new MoveElevatorToPreset(Elevator.PositionPresets.HATCH_LVL_2))));

        runAction(new HomeElevator());
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightLvl1ToRightRocket.left, true));
        runAction(new ShootHatch(true));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightRocketToLoadStation.left, false));
        runAction(new ShootHatch(false));
        runAction(new CollectHatch(true));
        runAction(new TurnInPlace(170.0));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightRocketToLoadStation2.left, false));
        runAction(new CollectHatch(false));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightLoadStationToRightRocket.left, false));
        runAction(new TurnInPlace(350.0));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightLoadStationToRightRocket2.left, false));
        runAction(new MoveElevatorToPreset(Elevator.PositionPresets.HATCH_LVL_2));
        runAction(new ShootHatch(true));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rightRocketToLoadStation.left, false));
    }
	
}