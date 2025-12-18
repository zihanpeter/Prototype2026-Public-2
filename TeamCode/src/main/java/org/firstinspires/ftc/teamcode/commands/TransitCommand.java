package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

/**
 * Command to handle the Transit (feeding) logic.
 * Controls the servo to feed rings into the shooter when conditions are met.
 */
public class TransitCommand extends CommandBase {
    private final Transit transit;
    private final Shooter shooter;
    
    // Timer for pulse logic (lifting servo for a set duration)
    private final com.qualcomm.robotcore.util.ElapsedTime pulseTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private boolean isPulseActive = false;

    /**
     * Constructor for TransitCommand.
     *
     * @param transit The transit subsystem.
     * @param intake The intake subsystem (currently unused in logic but kept for consistency).
     * @param shooter The shooter subsystem (used for velocity checks).
     */
    public TransitCommand(Transit transit, Intake intake, Shooter shooter) {
        this.transit = transit;
        this.shooter = shooter;
    }

    /**
     * Initialize command: Reset pulse state and ensure servo is down.
     */
    @Override
    public void initialize() {
        isPulseActive = false;
        transit.setTransitState(Transit.TransitState.DOWN);
    }

    /**
     * Execute logic:
     * Checks if shooter is at target velocity (within tolerance).
     * If FAST mode: Pulses the servo (Up for duration, then Down).
     * If MID/SLOW mode: Holds the servo UP as long as conditions are met.
     */
    @Override
    public void execute() {
        double currentVel = shooter.getVelocity();
        double targetVel = shooter.getTargetVelocity();
        double toleranceUpper = 300; // Default
        double toleranceLower = 300; // Default

        // Select tolerance based on current shooter state
        if (shooter.shooterState == Shooter.ShooterState.FAST) {
            toleranceUpper = ShooterConstants.toleranceFastUpper;
            toleranceLower = ShooterConstants.toleranceFastLower;
        } else if (shooter.shooterState == Shooter.ShooterState.MID) {
            toleranceUpper = ShooterConstants.toleranceMidUpper;
            toleranceLower = ShooterConstants.toleranceMidLower;
        } else if (shooter.shooterState == Shooter.ShooterState.SLOW) {
            toleranceUpper = ShooterConstants.toleranceSlowUpper;
            toleranceLower = ShooterConstants.toleranceSlowLower;
        }

        // Check if velocity is within tolerance range
        // Since velocities are negative:
        // Upper Bound (Slower/Less Negative): targetVel + toleranceUpper
        // Lower Bound (Faster/More Negative): targetVel - toleranceLower
        // Condition: (targetVel - toleranceLower) <= currentVel <= (targetVel + toleranceUpper)
        boolean inRange = (currentVel >= (targetVel - toleranceLower)) && (currentVel <= (targetVel + toleranceUpper));

        if (inRange && shooter.shooterState != Shooter.ShooterState.STOP) {
            // Apply pulse logic ONLY for FAST state
            if (shooter.shooterState == Shooter.ShooterState.FAST) {
                if (!isPulseActive) {
                    // Start Pulse
                    isPulseActive = true;
                    pulseTimer.reset();
                    transit.setTransitState(Transit.TransitState.UP);
                } else {
                    // Check Pulse Duration
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

    /**
     * End command: Ensure servo is retracted (DOWN).
     */
    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.DOWN);
    }
}
