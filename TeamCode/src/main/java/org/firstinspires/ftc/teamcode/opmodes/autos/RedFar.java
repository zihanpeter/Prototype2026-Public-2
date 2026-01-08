package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoAlignCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Autonomous OpMode for the Red Alliance (Far Side).
 * Mirrored from BlueFar (X=72 axis).
 * 6 paths (3 push cycles), all BezierLine (straight lines).
 */
@Autonomous(name = "Red Far Auto", group = "Far")
public class RedFar extends AutoCommandBase {

    // Path chains (6 paths - 1 push cycle per loop + park)
    private PathChain path1_toShootPose;
    private PathChain path2_toSample1;
    private PathChain path3_toPushZone;
    private PathChain path4_toSample2;
    private PathChain path5_toShootPose;
    private PathChain path6_toPark;

    // Poses mirrored from BlueFar (X=72 axis)
    // Blue: (55.269, 7.953, 90°) → Red: (88.731, 7.953, 90°)
    private final Pose startPose = new Pose(88.731, 7.953, Math.toRadians(90));
    // Blue: (61, 11.890, 112.5°) → Red: (83, 11.890, 67.5°)
    private final Pose shootPose = new Pose(83, 11.890, Math.toRadians(67.5));
    // Blue: (7, 10.569, 180°) → Red: (137, 10.569, 0°)
    private final Pose sample1Pose = new Pose(137, 10.569, Math.toRadians(0));
    // Blue: (30.679, 10.569, 180°) → Red: (113.321, 10.569, 0°)
    private final Pose pushZone = new Pose(113.321, 10.569, Math.toRadians(0));
    // Blue: (11.450, 10.569, 180°) → Red: (132.550, 10.569, 0°)
    private final Pose sample2Pose = new Pose(132.550, 10.569, Math.toRadians(0));
    // Blue: (58.716, 11.743, 112.5°) → Red: (85.284, 11.743, 67.5°)
    private final Pose finalShootPose = new Pose(85.284, 11.743, Math.toRadians(67.5));
    // Blue: (38.86, 12.094, 90°) → Red: (105.14, 12.094, 90°)
    private final Pose parkPose = new Pose(105.14, 12.094064949608057, Math.toRadians(90));

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        // Single cycle sequence: 1 round trip + shoot
        Command pushAndShootCycle = new SequentialCommandGroup(
                // Path 2: Shoot Pose -> Sample 1
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFastShooting(false);
                }),
                new AutoDriveCommand(follower, path2_toSample1, 4000),

                // Path 3: Sample 1 -> Push Zone
                new AutoDriveCommand(follower, path3_toPushZone, 3000),

                // Path 4: Push Zone -> Sample 2
                new AutoDriveCommand(follower, path4_toSample2, 3000),

                // Path 5: Sample 2 -> Shoot Pose
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.FAST);
                    intake.setFastShooting(true);
                }),
                new AutoDriveCommand(follower, path5_toShootPose, 4000),
                new AutoAlignCommand(follower, vision).withTimeout(1000),  // Auto-aim before shooting
                new TransitCommand(transit, shooter).withTimeout(1300)
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
                new AutoDriveCommand(follower, path1_toShootPose, 3000),
                new AutoAlignCommand(follower, vision).withTimeout(1000),  // Auto-aim before shooting
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // Loop 1
                // =========================================================
                pushAndShootCycle,

                // =========================================================
                // Loop 2
                // =========================================================
                pushAndShootCycle,

                // =========================================================
                // Loop 3
                // =========================================================
                pushAndShootCycle,

                // =========================================================
                // Park
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFastShooting(false);
                }),
                new AutoDriveCommand(follower, path6_toPark, 3000)
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Shoot Pose (BezierLine)
        // Red: 90° -> 67.5°
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67.5))
                .build();

        // Path 2: Shoot Pose -> Sample 1 (BezierLine)
        // Red: 67.5° -> 0°
        path2_toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, sample1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(67.5), Math.toRadians(0))
                .build();

        // Path 3: Sample 1 -> Push Zone (BezierLine)
        // Red: 0° -> 0°
        path3_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, pushZone))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 4: Push Zone -> Sample 2 (BezierLine)
        // Red: 0° -> 0°
        path4_toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone, sample2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 5: Sample 2 -> Shoot Pose (BezierLine)
        // Red: 0° -> 67.5°
        path5_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, finalShootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67.5))
                .build();

        // Path 6: Shoot Pose -> Park (BezierLine)
        // Red: 67.5° -> 90°
        path6_toPark = follower.pathBuilder()
                .addPath(new BezierLine(finalShootPose, parkPose))
                .setLinearHeadingInterpolation(Math.toRadians(67.5), Math.toRadians(90))
                .build();
    }
}
