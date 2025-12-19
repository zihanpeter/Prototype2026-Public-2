package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Transit subsystem.
 */
@Config
public class TransitConstants {
    // Hardware map name for the transit servo
    public static String transitServoName = "transitServo";

    // Servo positions
    public static double transitUpPos = 0.94;   // Position to push element into shooter
    public static double transitDownPos = 0.74; // Retracted position

    // Duration (in seconds) for the pulse logic (used in FAST mode)
    // Servo lifts for this duration then retracts
    public static double pulseDuration = 0.25;
}
