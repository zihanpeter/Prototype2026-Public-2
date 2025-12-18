package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Sample Blue Alliance Autonomous OpMode.
 * Demonstrates basic usage of the AutoCommandBase structure.
 */
@Config
@Autonomous(name = "Blue Sample Auto", group = "Autos")
public class BlueSampleAuto extends AutoCommandBase {
    public PathChain path1;

    @Override
    public Pose getStartPose() {
        // Define starting pose: (0, 0, 0 radians)
        return new Pose(0, 0, Math.toRadians(0));
    }

    @Override
    public Command runAutoCommand() {
        // Define a simple path: Move forward 24 inches
        path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(0, 0), new Pose(24, 0))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Return a sequence of commands
        return new SequentialCommandGroup(
                // 1. Spin up Shooter to MID speed
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.MID)),
                
                // 2. Drive along the path
                new AutoDriveCommand(follower, path1),
                
                // 3. Stop Shooter after path completion
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }
}
