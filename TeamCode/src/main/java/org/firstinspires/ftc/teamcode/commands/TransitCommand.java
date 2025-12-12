package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

public class TransitCommand extends CommandBase {
    private final Transit transit;

    private final Shooter shooter;
    
    private final com.qualcomm.robotcore.util.ElapsedTime pulseTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private boolean isPulseActive = false;

    public TransitCommand(Transit transit, Intake intake, Shooter shooter) {
        this.transit = transit;

        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        isPulseActive = false;
        transit.setTransitState(Transit.TransitState.DOWN);
    }

    @Override
    public void execute() {
        // If the shooter is up to speed and spinning, lift the transit servo to feed the ring
        // Added tolerance: Only lift if velocity is within a range of the target
        // Target is negative (e.g. -1500). Current is negative (e.g. -1600).
        // Check if current is MORE negative than target (faster) AND not TOO much faster (safety?)
        // Or just stick to "Fast enough" logic.
        // User request: "Only operate servo within a certain +/- range of target speed"
        
        double currentVel = shooter.getVelocity();
        double targetVel = shooter.getTargetVelocity();
        double tolerance = ShooterConstants.toleranceMid; // Default

        if (shooter.shooterState == Shooter.ShooterState.FAST) {
            tolerance = ShooterConstants.toleranceFast;
        } else if (shooter.shooterState == Shooter.ShooterState.MID) {
            tolerance = ShooterConstants.toleranceMid;
        } else if (shooter.shooterState == Shooter.ShooterState.SLOW) {
            tolerance = ShooterConstants.toleranceSlow;
        }

        boolean inRange = Math.abs(currentVel - targetVel) <= tolerance;

        if (inRange && shooter.shooterState != Shooter.ShooterState.STOP) {
            // Apply pulse logic ONLY for FAST state
            if (shooter.shooterState == Shooter.ShooterState.FAST) {
                if (!isPulseActive) {
                    isPulseActive = true;
                    pulseTimer.reset();
                    transit.setTransitState(Transit.TransitState.UP);
                } else {
                    if (pulseTimer.seconds() > TransitConstants.pulseDuration) {
                        transit.setTransitState(Transit.TransitState.DOWN);
                    } else {
                        transit.setTransitState(Transit.TransitState.UP);
                    }
                }
            } else {
                // For MID and SLOW, use continuous lift (hold to shoot)
            transit.setTransitState(Transit.TransitState.UP);
                isPulseActive = false; // Reset pulse state just in case
            }
        } else {
            // Speed dropped or shooter stopped, reset pulse state and lower servo
            isPulseActive = false;
            transit.setTransitState(Transit.TransitState.DOWN);
        }


    }

    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.DOWN);

    }
}
