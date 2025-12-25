package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Command for auto-aligning to a goal AprilTag (20 or 24) in autonomous.
 * Uses Limelight's tx to calculate target heading, then uses Follower.holdPoint to turn.
 * Finishes when aligned (tx < threshold) or after timeout.
 */
public class AutoAlignCommand extends CommandBase {
    private final Follower follower;
    private final Vision vision;
    
    // Alignment parameters
    private static final double TX_THRESHOLD = 2.0; // Degrees - considered aligned when |tx| < this
    
    private int alignedFrameCount = 0;
    private static final int REQUIRED_ALIGNED_FRAMES = 5;  // Need 5 consecutive aligned frames
    
    /**
     * Creates an AutoAlignCommand.
     * @param follower The PedroPathing Follower instance.
     * @param vision The Vision subsystem.
     */
    public AutoAlignCommand(Follower follower, Vision vision) {
        this.follower = follower;
        this.vision = vision;
    }
    
    @Override
    public void initialize() {
        alignedFrameCount = 0;
        // Stop any path following first
        follower.breakFollowing();
    }
    
    @Override
    public void execute() {
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        Pose currentPose = follower.getPose();
        
        if (!isGoalTag) {
            // No goal tag in view, hold current position
            follower.holdPoint(currentPose);
            follower.update();
            alignedFrameCount = 0;
            return;
        }
        
        double tx = vision.getTx();
        
        // Check if aligned
        if (Math.abs(tx) < TX_THRESHOLD) {
            alignedFrameCount++;
            // Hold current position when aligned
            follower.holdPoint(currentPose);
        } else {
            alignedFrameCount = 0;
            
            // Calculate target heading based on tx
            // tx > 0 means target is to the right, so turn right (decrease heading)
            double targetHeading = currentPose.getHeading() - Math.toRadians(tx);
            
            // Create target pose with same X/Y but new heading
            Pose targetPose = new Pose(currentPose.getX(), currentPose.getY(), targetHeading);
            follower.holdPoint(targetPose);
        }
        
        follower.update();
    }
    
    @Override
    public boolean isFinished() {
        return alignedFrameCount >= REQUIRED_ALIGNED_FRAMES;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Hold final position
        follower.holdPoint(follower.getPose());
        follower.update();
    }
}

