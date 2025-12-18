package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.subsystems.transit.TransitConstants;

/**
 * Command to control the Transit servo based on Shooter velocity.
 * "Smart Fire": Only pushes the ring when shooter is at target velocity.
 */
public class TransitCommand extends CommandBase {
    private final Transit transit;
    private final Shooter shooter;
    private final ElapsedTime pulseTimer = new ElapsedTime();
    private boolean isPulseActive = false;

    public TransitCommand(Transit transit, Shooter shooter) {
        this.transit = transit;
        this.shooter = shooter;
        addRequirements(transit); // Only requires Transit, Shooter is read-only here
    }

    /**
     * Called repeatedly while the command is scheduled (usually while Trigger is held).
     */
    @Override
    public void execute() {
        double currentVel = shooter.getVelocity();
        double targetVel = shooter.getTargetVelocity();
        double toleranceUpper = ShooterConstants.toleranceMidUpper; // Default
        double toleranceLower = ShooterConstants.toleranceMidLower; // Default

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
        // Note: Velocities are negative, so (target - lower) is actually a "higher" speed (more negative) if we look at magnitude
        // Let's stick to simple algebraic range: [target - lower, target + upper]
        boolean inRange = (currentVel <= targetVel + toleranceUpper) && (currentVel >= targetVel - toleranceLower);

        // For STOP state, we don't shoot
        if (shooter.shooterState == Shooter.ShooterState.STOP) {
            inRange = false;
        }

        if (inRange) {
            // Logic: If in range, push the ring up
            // Use pulse for FAST mode to avoid jamming? Or just hold for now.
            // Let's implement the Pulse Logic from previous Context
            
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
                // For MID/SLOW, just hold UP
                transit.setTransitState(Transit.TransitState.UP);
                isPulseActive = false;
            }
        } else {
            // Not in range, keep down
            isPulseActive = false;
            transit.setTransitState(Transit.TransitState.DOWN);
        }
    }

    @Override
    public boolean isFinished() {
        // For Auto: You might want this command to finish after a certain time or condition?
        // But usually TransitCommand is a "while active" command or triggered instantly.
        // If used in SequentialCommandGroup, it needs to finish!
        
        // AUTO MODE FIX:
        // If this command is used in Auto sequence (new TransitCommand(...)), it acts as a "Shoot Once" action.
        // We should wait until we have successfully pulsed/held for a duration.
        
        // Let's assume if we are in Pulse mode and timer > duration + cushion, we are done.
        // Or for simplicity in Auto, we can just use a Timeout decoration externally.
        // But to make it standalone:
        
        if (isPulseActive && pulseTimer.seconds() > TransitConstants.pulseDuration + 0.2) {
             return true; 
        }
        
        // For non-pulse modes (MID), we need a timer too to know when "shot is done"
        // Let's add a simple internal timer for the whole command
        return false; // Rely on external .withTimeout() or ParallelRaceGroup
    }

    /**
     * Called when the command ends (Trigger released).
     * Resets mechanisms to default state.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        // Reset transit servo to DOWN
        transit.setTransitState(Transit.TransitState.DOWN);
        isPulseActive = false;
    }
}
