package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/**
 * Command for auto-aligning to a goal AprilTag (20 or 24) in autonomous.
 * Uses Limelight's tx to turn towards the tag.
 * Finishes when aligned (tx < threshold) or after timeout.
 */
public class AutoAlignCommand extends CommandBase {
    private final MecanumDrivePinpoint drive;
    private final Vision vision;
    
    // Alignment parameters
    private static final double kP = 0.03;           // Proportional gain for tx alignment
    private static final double TX_THRESHOLD = 2.0; // Degrees - considered aligned when tx < this
    private static final double MIN_POWER = 0.1;    // Minimum turn power to overcome static friction
    
    private int alignedFrameCount = 0;
    private static final int REQUIRED_ALIGNED_FRAMES = 3;  // Need 3 consecutive aligned frames
    
    /**
     * Creates an AutoAlignCommand.
     * @param drive The MecanumDrivePinpoint subsystem.
     * @param vision The Vision subsystem.
     */
    public AutoAlignCommand(MecanumDrivePinpoint drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        alignedFrameCount = 0;
    }
    
    @Override
    public void execute() {
        int tagId = vision.getDetectedTagId();
        boolean isGoalTag = (tagId == Vision.BLUE_GOAL_TAG_ID || tagId == Vision.RED_GOAL_TAG_ID);
        
        if (!isGoalTag) {
            // No goal tag in view, stop and wait
            drive.moveRobot(0, 0, 0);
            alignedFrameCount = 0;  // Reset counter
            return;
        }
        
        double tx = vision.getTx();
        
        // Check if aligned
        if (Math.abs(tx) < TX_THRESHOLD) {
            alignedFrameCount++;
            drive.moveRobot(0, 0, 0);  // Stop when aligned
        } else {
            alignedFrameCount = 0;  // Reset counter if not aligned
            
            // Calculate turn power
            double turn = tx * kP;
            
            // Apply minimum power to overcome static friction
            if (Math.abs(turn) < MIN_POWER && Math.abs(tx) > 0.5) {
                turn = Math.signum(turn) * MIN_POWER;
            }
            
            // Clamp turn power
            turn = Math.max(-1, Math.min(1, turn));
            
            // Turn robot (no forward/strafe movement)
            drive.moveRobot(0, 0, turn);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish when aligned for enough frames
        return alignedFrameCount >= REQUIRED_ALIGNED_FRAMES;
    }
    
    @Override
    public void end(boolean interrupted) {
        drive.moveRobot(0, 0, 0);  // Stop robot
    }
}

