package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Command to manually control the Intake.
 * Note: Intake is largely automated (always on), so this command might be redundant
 * or used for specific manual overrides/toggles.
 */
public class IntakeCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;

    public IntakeCommand(Transit transit, Intake intake) {
        this.transit = transit;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        // Ensure transit is down when starting manual intake
        transit.setTransitState(Transit.TransitState.DOWN);
    }

    @Override
    public void execute() {
        // Ensure intake is running if it isn't already
        if (!intake.isRunning()) {
            intake.toggle();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Logic to stop intake on command end is commented out to support "Always On" behavior
        // if (intake.isRunning()) {
        //    intake.toggle();
        // }
    }
}
