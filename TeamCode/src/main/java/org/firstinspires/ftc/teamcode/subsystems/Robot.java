package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Robot Hardware Container.
 * Initializes and holds references to all robot subsystems.
 * Simplifies dependency injection in OpModes.
 */
public class Robot {
    public final MecanumDrivePinpoint drive;
    public final Shooter shooter;
    public final Transit transit;
    public final Intake intake;

    /**
     * Constructor for Robot.
     * Initializes all subsystems using the provided HardwareMap.
     *
     * @param hardwareMap The hardware map from the OpMode.
     */
    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrivePinpoint(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);
    }
}

