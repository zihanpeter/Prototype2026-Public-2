package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Util {
    public static Pose Pose2DToPose(Pose2D pose2D) {
        return new Pose(pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS));
    }

    public static boolean epsilonEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }
    
    /**
     * Converts Limelight Pose3D (meters, field-space) to Pinpoint Pose2D (inches).
     * Limelight uses a different coordinate system, so we need to transform:
     * - Limelight Y -> Pinpoint X (+ 72 to shift origin to field center)
     * - Limelight X -> Pinpoint Y (negated, + 72 to shift origin)
     * - Limelight Yaw (degrees) -> Pinpoint Heading (radians, - PI/2 for rotation offset)
     *
     * @param pose3D The Pose3D from Limelight (in meters and degrees)
     * @return Pose2D for Pinpoint (in inches and radians)
     */
    public static Pose2D visionPoseToPinpointPose(Pose3D pose3D) {
        return new Pose2D(
                DistanceUnit.INCH,
                Units.metersToInches(pose3D.getPosition().y) + 72,   // Limelight Y -> X
                -Units.metersToInches(pose3D.getPosition().x) + 72,  // Limelight X -> Y (negated)
                AngleUnit.RADIANS,
                pose3D.getOrientation().getYaw() / 180.0 * Math.PI - Math.PI / 2  // Degrees -> Radians with offset
        );
    }
}
