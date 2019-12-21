package com.team4911.frc2019.paths;

import com.team254.lib.geometry.Pose2d;
import org.junit.jupiter.api.Test;
import static com.team4911.frc2019.paths.FieldPoses.kSafeDistance;
//import static org.junit.jupiter.api.Assertions.*;

class FieldPosesTest {

    @Test
    void test() {
        System.out.println(FieldPoses.kFeederLeft.toString() + " -22 = " + FieldPoses.getSafePose(FieldPoses.kFeederLeft).toString());
        System.out.println(FieldPoses.kFeederRight.toString() + " -22 = " + FieldPoses.getSafePose(FieldPoses.kFeederRight).toString());
        System.out.println(FieldPoses.kRocketLeftNear.toString() + " -22 = " + FieldPoses.getSafePose(FieldPoses.kRocketLeftNear).toString());

        System.out.println(FieldPoses.kRocketLeftFar.toString() + " -22 = " + FieldPoses.getSafePose(FieldPoses.kRocketLeftFar).toString());

        Pose2d pose = FieldPoses.kRocketLeftNear.transformBy(kSafeDistance);
        System.out.println(FieldPoses.kRocketLeftNear.toString() + " -22 = " + pose.toString());
        System.out.println("Distance=" + pose.distance(FieldPoses.kRocketLeftNear));
    }
}