package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoAlignCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Autonomous OpMode for the Red Alliance (Far Side).
 * Mirrored from BlueFar (X=72 axis).
 * 6 paths (2 push cycles), all BezierLine (straight lines).
 */
@Autonomous(name = "Red Far Auto", group = "Red")
public class RedFar extends AutoCommandBase {

    // Path chains (5 paths - 1 push cycle per loop)
    private PathChain path1_toShootPose;
    private PathChain path2_toSample1;
    private PathChain path3_toPushZone;
    private PathChain path4_toSample2;
    private PathChain path5_toShootPose;

    // Poses mirrored from Blue (X=72 axis)
    // Blue: (55.269, 7.953, 90°) → Red: (88.731, 7.953, 90°)
    private final Pose startPose = new Pose(88.731, 7.953, Math.toRadians(90));
    // Blue: (58.716, 11.890, 114°) → Red: (85.284, 11.890, 66°)
    private final Pose shootPose = new Pose(85.284, 11.890, Math.toRadians(66));
    // Blue: (11.303, 10.569, 180°) → Red: (132.697, 10.569, 0°)
    private final Pose sample1Pose = new Pose(132.697, 10.569, Math.toRadians(0));
    // Blue: (30.679, 10.569, 180°) → Red: (113.321, 10.569, 0°)
    private final Pose pushZone = new Pose(113.321, 10.569, Math.toRadians(0));
    // Blue: (11.450, 10.569, 180°) → Red: (132.550, 10.569, 0°)
    private final Pose sample2Pose = new Pose(132.550, 10.569, Math.toRadians(0));
    // Blue: (58.716, 11.743, 114°) → Red: (85.284, 11.743, 66°)
    private final Pose finalShootPose = new Pose(85.284, 11.743, Math.toRadians(66));

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        // 循环序列：往返1次 + 射击（无限重复）
        Command pushAndShootLoop = new RepeatCommand(
                new SequentialCommandGroup(
                        // Path 2: Shoot Pose -> Sample 1
                        new InstantCommand(() -> {
                            shooter.setShooterState(Shooter.ShooterState.STOP);
                            intake.setFastShooting(false);
                        }),
                        new AutoDriveCommand(follower, path2_toSample1),

                        // Path 3: Sample 1 -> Push Zone
                        new AutoDriveCommand(follower, path3_toPushZone),

                        // Path 4: Push Zone -> Sample 2
                        new AutoDriveCommand(follower, path4_toSample2),

                        // Path 5: Sample 2 -> Shoot Pose
                        new InstantCommand(() -> {
                            shooter.setShooterState(Shooter.ShooterState.FAST);
                            intake.setFastShooting(true);
                        }),
                        new AutoDriveCommand(follower, path5_toShootPose),
                        new AutoAlignCommand(drive, vision).withTimeout(1000),  // Auto-aim before shooting
                        new TransitCommand(transit, shooter) // Shoots 3 times then finishes
                )
        );

        return new SequentialCommandGroup(
                // =========================================================
                // Initialize: Start intake
                // =========================================================
                new InstantCommand(() -> intake.startIntake()),

                // =========================================================
                // 1. Path 1: Start -> Shoot Pose (Preload)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.FAST);
                    intake.setFastShooting(true);
                }),
                new AutoDriveCommand(follower, path1_toShootPose),
                new AutoAlignCommand(drive, vision).withTimeout(1000),  // Auto-aim before shooting
                new TransitCommand(transit, shooter), // Shoots 3 times then finishes

                // =========================================================
                // 2-6. Infinite Loop: 往返2次 + 射击
                // =========================================================
                pushAndShootLoop
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Shoot Pose (BezierLine)
        // Blue: 90° -> 114° → Red: 90° -> 66°
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))
                .build();

        // Path 2: Shoot Pose -> Sample 1 (BezierLine)
        // Blue: 114° -> 180° → Red: 66° -> 0°
        path2_toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, sample1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .build();

        // Path 3: Sample 1 -> Push Zone (BezierLine)
        // Blue: 180° -> 180° → Red: 0° -> 0°
        path3_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, pushZone))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 4: Push Zone -> Sample 2 (BezierLine)
        // Blue: 180° -> 180° → Red: 0° -> 0°
        path4_toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone, sample2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 5: Sample 2 -> Shoot Pose (BezierLine)
        // Blue: 180° -> 114° → Red: 0° -> 66°
        path5_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, finalShootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();
    }
}
