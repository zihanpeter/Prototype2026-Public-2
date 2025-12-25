package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

/**
 * Command to hold the robot's current position using PedroPathing's Follower.
 * Used during shooting to prevent drift.
 * This command never finishes on its own - use .withTimeout() or as part of ParallelDeadlineGroup.
 */
public class HoldPositionCommand extends CommandBase {
    private final Follower follower;
    private Pose holdPose;
    
    public HoldPositionCommand(Follower follower) {
        this.follower = follower;
    }
    
    @Override
    public void initialize() {
        // Capture current pose when command starts
        holdPose = follower.getPose();
    }
    
    @Override
    public void execute() {
        // Continuously hold the position
        follower.holdPoint(holdPose);
        follower.update();
    }
    
    @Override
    public boolean isFinished() {
        // Never finishes on its own
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Keep holding the last position
    }
}

