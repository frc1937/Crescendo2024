package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.math.Conversions;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

class ButtonTest {

    @Test
    void testButton() {
        DriverStation.silenceJoystickConnectionWarning(true);

        double rpm = Conversions.RPMFromTangentialVelocity(MetersPerSecond.of(2), Inches.of(4));

        System.out.println(rpm  );


        Map<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();

        for (AprilTag aprilTag :  AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTags())
            TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);


//        for (AprilTagFieldLayout.TagPosition tagPosition : AprilTagFieldLayout()) {
            System.out.println("Y:" + TAG_ID_TO_POSE.get(7).getY()); //Y:5.547867999999999
            System.out.println("X:" + TAG_ID_TO_POSE.get(7).getX()); //X:-0.038099999999999995
//        }
    }
}
