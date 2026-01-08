package org.firstinspires.ftc.teamcode.utils;

/**
 * Converter from FTC SDK Pose2D to Pedro Pathing Pose.
 */

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pose2dToPose {
    public static Pose convert(Pose2D pose2D) {
        return new Pose(pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS));
    }
}
