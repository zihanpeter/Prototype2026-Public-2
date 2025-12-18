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
    }

    /**
     * Update the follower loop.
     */
    @Override
    public void execute() {
        follower.update();
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
     * Command finishes when the follower is no longer busy (path completed).
     */
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
