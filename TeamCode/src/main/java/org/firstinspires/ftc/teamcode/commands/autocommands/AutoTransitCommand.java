package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Command to control Transit servo in autonomous with position check.
 * Only allows Transit to go UP when:
 * 1. Robot is at the target position (within tolerance)
 * 2. Shooter is at target velocity
 * Also controls Intake: only runs at 0.8 power when at position.
 */
public class AutoTransitCommand extends CommandBase {
    private final Transit transit;
    private final Shooter shooter;
    private final Follower follower;
    private final Pose targetPose;
    private final Intake intake;
    
    // Position tolerances (relaxed for testing)
    private static final double POSITION_TOLERANCE = 10.0;  // inches (relaxed from 2.0)
    private static final double HEADING_TOLERANCE = Math.toRadians(45);  // 45 degrees (relaxed from 20)
    
    /**
     * Creates an AutoTransitCommand with position check.
     * @param transit The Transit subsystem
     * @param shooter The Shooter subsystem
     * @param follower The PedroPathing Follower
     * @param targetPose The target shooting position
     * @param intake The Intake subsystem
     */
    public AutoTransitCommand(Transit transit, Shooter shooter, Follower follower, Pose targetPose, Intake intake) {
        this.transit = transit;
        this.shooter = shooter;
        this.follower = follower;
        this.targetPose = targetPose;
        this.intake = intake;
    }
    
    @Override
    public void execute() {
        // Check if robot is at target position
        boolean atPosition = isAtTargetPosition();
        
        // Check if shooter is ready
        boolean shooterReady = shooter.isShooterAtSetPoint();
        
        // Control intake: only fast shooting when at position
        intake.setFastShooting(atPosition);
        
        // Only raise transit if both conditions are met
        if (atPosition && shooterReady) {
            transit.setTransitState(Transit.TransitState.UP);
        } else {
            transit.setTransitState(Transit.TransitState.DOWN);
        }
    }
    
    /**
     * Checks if the robot is within tolerance of the target position.
     * @return true if at target position
     */
    private boolean isAtTargetPosition() {
        Pose currentPose = follower.getPose();
        
        double dx = Math.abs(currentPose.getX() - targetPose.getX());
        double dy = Math.abs(currentPose.getY() - targetPose.getY());
        double dh = Math.abs(normalizeAngle(currentPose.getHeading() - targetPose.getHeading()));
        
        return dx < POSITION_TOLERANCE && dy < POSITION_TOLERANCE && dh < HEADING_TOLERANCE;
    }
    
    /**
     * Normalizes angle to [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    @Override
    public boolean isFinished() {
        // Never finishes on its own - use .withTimeout()
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        transit.setTransitState(Transit.TransitState.DOWN);
        intake.setFastShooting(false);  // Reset intake power when done
    }
}

