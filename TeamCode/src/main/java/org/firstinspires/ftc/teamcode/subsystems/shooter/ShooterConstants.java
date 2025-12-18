package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

/**
 * Constants for the Shooter subsystem.
 */
@Config
public class ShooterConstants {
    // Hardware map names
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String shooterServoName = "shooterServo";

    // PID Tolerance (ticks) - Used for telemetry or checks
    public static double shooterEpsilon = 20;

    // PID Coefficients (Currently unused for main control loop)
    public static double kP = 0.5;
    public static double kI = 0;
    public static double kD = 0;

    // Theoretical Max TPS for Feedforward Calculation
    // GoBilda 6000RPM Motor (5203-2402-0001): 28 ticks/revolution
    // Max TPS = (6000 / 60) * 28 = 2800 TPS
    public static double maxVelocityTPS = 2800.0;

    /**
     * Target Velocities (in Ticks Per Second)
     * Negative values indicate direction.
     */
    public static double stopVelocity = -900;  // Idle speed (Same as slow/close range)
    public static double fastVelocity = -1250; // Updated for 6000RPM motor (~75% power)
    public static double midVelocity = -1150;  // Updated for mid-range shots (~46% power)
    public static double slowVelocity = -600;  // Low speed for close/safe shots (~21% power)
    public static double releaseVelocity = -200; // Threshold to consider "stopped" or "too slow"
    
    // Velocity tolerances for transit engagement (ticks per second)
    // Defines the acceptable range around the target velocity.
    // Upper: Max allowed speed ABOVE target (less negative magnitude)
    // Lower: Max allowed speed BELOW target (more negative magnitude)
    
    // MID Tolerances
    public static double toleranceMidUpper = 50; // Allow being slightly slower
    public static double toleranceMidLower = 50; // Allow being slightly faster
    
    // FAST Tolerances
    public static double toleranceFastUpper = 50;
    public static double toleranceFastLower = 50; // Usually okay to be faster
    
    // SLOW Tolerances
    public static double toleranceSlowUpper = 400;
    public static double toleranceSlowLower = 300;

    // Servo Positions for Angle Adjustment
    // Updated based on user request: Close (0.85) -> Far (0.29)
    public static double shooterServoUpPos = 0.29;   // Position for FAST/Long range
    public static double shooterServoMidPos = 0.57;  // Position for MID range (Calculated average)
    public static double shooterServoDownPos = 0.85; // Position for SLOW/Short range or Stowed
}
