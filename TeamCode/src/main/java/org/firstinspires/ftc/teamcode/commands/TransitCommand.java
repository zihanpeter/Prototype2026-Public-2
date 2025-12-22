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
    private final ElapsedTime cooldownTimer = new ElapsedTime();
    private final ElapsedTime finishDelayTimer = new ElapsedTime();
    private boolean isPulseActive = false;
    private boolean isCoolingDown = false;
    private int shotCount = 0; // Count of completed shots (for FAST mode)
    private boolean isFinishing = false; // Whether we've completed required shots and are waiting for delay
    private static final double COOLDOWN_DURATION = 0.5; // 0.5 second cooldown after FAST shot
    private static final int REQUIRED_SHOTS_FAST = 3; // Number of shots required for FAST mode
    private static final double FINISH_DELAY = 0.2; // Delay after completing shots

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

        // Handle cooldown period (FAST mode only)
        if (isCoolingDown) {
            if (cooldownTimer.seconds() > COOLDOWN_DURATION) {
                isCoolingDown = false; // Cooldown finished
            } else {
                transit.setTransitState(Transit.TransitState.DOWN);
                return; // Still cooling down, don't shoot
            }
        }

        if (inRange) {
            // Logic: If in range, push the ring up
            // Use pulse for FAST mode to avoid jamming
            
            if (shooter.shooterState == Shooter.ShooterState.FAST) {
                // Check if we've already completed required shots
                if (shotCount >= REQUIRED_SHOTS_FAST) {
                    transit.setTransitState(Transit.TransitState.DOWN);
                    if (!isFinishing) {
                        isFinishing = true;
                        finishDelayTimer.reset();
                    }
                    return; // Don't shoot anymore
                }
                
                if (!isPulseActive) {
                    isPulseActive = true;
                    pulseTimer.reset();
                    transit.setTransitState(Transit.TransitState.UP);
                } else {
                    if (pulseTimer.seconds() > TransitConstants.pulseDuration) {
                        // Pulse finished, start cooldown
                        transit.setTransitState(Transit.TransitState.DOWN);
                        isPulseActive = false;
                        isCoolingDown = true;
                        cooldownTimer.reset();
                        shotCount++; // Increment shot count
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
        // FAST mode: Finish after completing REQUIRED_SHOTS_FAST shots + delay
        if (shooter.shooterState == Shooter.ShooterState.FAST) {
            if (isFinishing && finishDelayTimer.seconds() > FINISH_DELAY) {
                return true;
            }
            return false;
        }
        
        // For MID/SLOW modes: Rely on external .withTimeout()
        return false;
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
        isCoolingDown = false;
        shotCount = 0;
        isFinishing = false;
    }
}
