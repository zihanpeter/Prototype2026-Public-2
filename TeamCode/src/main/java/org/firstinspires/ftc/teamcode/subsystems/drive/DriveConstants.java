package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.Units;

/**
 * Constants for the Drive subsystem.
 * Includes motor names, localization offsets, and drive behavior parameters.
 */
@Config
public class DriveConstants {
    // Motor hardware map names
    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";

    // Pinpoint (Dead Wheels) Offsets
    // These values represent the position of the odometry pods relative to the robot's center.
    // xPoseDW corresponds to the Strafe Pod's X offset.
    // yPoseDW corresponds to the Forward Pod's Y offset.
    // Values are converted from mm (measured) to inches.
    public static double xPoseDW = 32.0 / 25.4, yPoseDW = -171.5 / 25.4;
    
    // OTOS Offsets (Legacy/Alternative)
    public static double xPoseOTOS = Units.mmToInches(-173), yPoseOTOS = 0, headingPoseOTOS = -Math.PI / 2;

    // Drive behavior constants
    public static double strafingBalance = 1.1; // Multiplier to correct strafing drift/inefficiency
    public static double headingEpsilon = 0.1;  // Tolerance for heading checks (radians)
    public static DistanceUnit distanceUnit = DistanceUnit.INCH;
    public static AngleUnit angleUnit = AngleUnit.RADIANS;

    // Pedro Pathing / Tuning Constants (Empirically determined)
    public static double linearScalar = -1.1404723265, angularScalar = 0.99446;
    public static double forwardVelocity = 80.198152, strafeVelocity = 66.5127;
    public static double forwardAcceleration = -32.6419, strafeAcceleration = -95.1316;

    // PID Coefficients (if used for active control)
    public static double kP_xy = 0.02;
    public static double kP_h = -0.8;

    // Gamepad Input Deadband
    // Minimum input value required to trigger robot movement.
    public static double deadband = 0.03;
}
