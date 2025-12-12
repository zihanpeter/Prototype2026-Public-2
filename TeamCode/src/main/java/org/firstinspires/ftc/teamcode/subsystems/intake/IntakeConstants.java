package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    public static String intakeMotorName = "intakeMotor";

    public static double intakePower = 0.35;
    public static double fullPower = 1.0;

    public static double transitPower = 1;

    /**
     * GoBilda 5203-2402-0051 (1150RPM, 5.2:1 Ratio)
     * Resolution: 145.1 PPR
     * Max TPS = (1150 / 60) * 145.1 ≈ 2781 TPS
     */
    public static double maxVelocityTPS = 2781.0;

    // Threshold ratio for jamming detection (20% of max speed)
    public static double jammingThresholdRatio = 0.2;
    // Power to apply when jamming is detected
    public static double jammingPower = 0.5;
}
