package org.firstinspires.ftc.teamcode.commands.autocommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

/**
 * Command for Autonomous driving using Pedro Pathing.
 * Follows a specified PathChain until completion.
 */
public class AutoDriveCommand extends CommandBase {
    private Follower follower;
    private PathChain pathChain;
    private long finishTime = 0;
    private static final long WAIT_MS = 250; // Buffer time to stabilize after path ends

    /**
     * Constructor for AutoDriveCommand.
     *
     * @param follower The Pedro Pathing Follower instance.
     * @param pathChain The PathChain to follow.
     */
    public AutoDriveCommand(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.pathChain = pathChain;
    }

    /**
     * Start following the path.
     */
    @Override
    public void initialize() {
        follower.followPath(pathChain);
        finishTime = 0;
    }

    /**
     * Update the follower loop.
     */
    @Override
    public void execute() {
        // follower.update() is handled in AutoCommandBase main loop
        // to ensure continuous holding. We do nothing here but wait.
    }

    /**
     * Stop following if interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if (follower.isBusy()) {
            follower.breakFollowing();
        }
    }

    /**
     * Command finishes when the follower is no longer busy AND a buffer time has passed.
     */
    @Override
    public boolean isFinished() {
        if (follower.isBusy()) {
            finishTime = 0; // Reset timer if still busy
            return false;
        } else {
            // Path finished (or thinks it did)
            if (finishTime == 0) {
                finishTime = System.currentTimeMillis(); // Start timer
            }
            // Only finish if stable for WAIT_MS
            return System.currentTimeMillis() - finishTime > WAIT_MS;
        }
    }
}
