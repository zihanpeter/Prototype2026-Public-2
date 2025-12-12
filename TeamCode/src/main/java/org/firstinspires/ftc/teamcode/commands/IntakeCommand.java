package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class IntakeCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;

    public IntakeCommand(Transit transit, Intake intake) {
        this.transit = transit;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        transit.setTransitState(Transit.TransitState.DOWN);
    }

    @Override
    public void execute() {
        // Command logic modified: This command is now likely redundant if Intake runs by default.
        // Keeping it for now but if bound, it ensures intake is ON.
        if (!intake.isRunning()) {
            intake.toggle();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Do NOT stop intake when command ends (per new "always on" requirement)
        // Only stop if explicitly toggled off elsewhere or robot stop.
        // If this command was used for "Hold to run", ending it would stop the intake.
        // Since we removed the binding in TeleOpSolo, this end() logic is less critical but for safety:
        
        // if (intake.isRunning()) {
        //    intake.toggle();
        // }
    }
}
