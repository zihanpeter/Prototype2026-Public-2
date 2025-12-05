package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class TransitCommand extends CommandBase {
    private final Transit transit;
    private final Intake intake;
    private final Shooter shooter;

    public TransitCommand(Transit transit, Intake intake, Shooter shooter) {
        this.transit = transit;
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        if (shooter.isShooterAtSetPoint() && shooter.shooterState != Shooter.ShooterState.STOP) {
            transit.setTransitState(Transit.TransitState.UP);
        }

        if (!intake.isRunning()) intake.toggle();
        if (!intake.isShooting()) intake.toggleShooting();
    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.DOWN);
        if (intake.isRunning()) intake.toggle();
        if (intake.isShooting()) intake.toggleShooting();
    }
}
