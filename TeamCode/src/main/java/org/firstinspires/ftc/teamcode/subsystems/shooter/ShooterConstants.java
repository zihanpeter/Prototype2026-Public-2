package org.firstinspires.ftc.teamcode.subsystems.shooter;


import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static String leftShooterName = "leftShooterMotor";
    public static String rightShooterName = "rightShooterMotor";
    public static String shooterServoName = "shooterServo";

    public static double shooterEpsilon = 20;

    public static double kP = 0.5;
    public static double kI = 0;
    public static double kD = 0;

    public static double maxVelocityTPS = 2800.0;

    /**
     * In Ticks Per Second
     * GoBilda 6000RPM Motor (5203-2402-0001): 28 ticks/revolution
     * Max TPS = (6000 / 60) * 28 = 2800 TPS
     */
    public static double stopVelocity = 0;
    public static double fastVelocity = -1400; // ~80% power
    public static double midVelocity = -1250; // ~55% power
    public static double slowVelocity = -1000;
    public static double releaseVelocity = -200;
    
    // Velocity tolerance for transit engagement (+/- ticks per second)
    public static double toleranceMid = 300;
    public static double toleranceFast = 20;
    public static double toleranceSlow = 300;

    public static double shooterServoUpPos = 0.85;
    public static double shooterServoMidPos = 0.54; // Assuming halfway or specific mid value
    public static double shooterServoDownPos = 0.23;
}
