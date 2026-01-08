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
 * Autonomous OpMode for the Blue Alliance (Far Side).
 * Mirrored from RedFar (X=72 axis).
 * 6 paths (3 push cycles), all BezierLine (straight lines).
 */
@Autonomous(name = "Blue Far Auto", group = "Far")
public class BlueFar extends AutoCommandBase {

    // Path chains (6 paths - 1 push cycle per loop + park)
    private PathChain path1_toShootPose;
    private PathChain path2_toSample1;
    private PathChain path3_toPushZone;
    private PathChain path4_toSample2;
    private PathChain path5_toShootPose;
    private PathChain path6_toPark;

    // Poses mirrored from Red (X=72 axis)
    // Red: (88.731, 7.953, 90°) → Blue: (55.269, 7.953, 90°)
    private final Pose startPose = new Pose(55.269, 7.953, Math.toRadians(90));
    // Red: (85.284, 11.890, 66°) → Blue: (58.716, 11.890, 112.5°)
    private final Pose shootPose = new Pose(61, 11.890, Math.toRadians(112.5));
    // Red: (137, 10.569, 0°) → Blue: (7, 10.569, 180°)
    private final Pose sample1Pose = new Pose(7, 10.569, Math.toRadians(180));
    // Red: (113.321, 10.569, 0°) → Blue: (30.679, 10.569, 180°)
    private final Pose pushZone = new Pose(30.679, 10.569, Math.toRadians(180));
    // Red: (132.550, 10.569, 0°) → Blue: (11.450, 10.569, 180°)
    private final Pose sample2Pose = new Pose(11.450, 10.569, Math.toRadians(180));
    // Red: (85.284, 11.743, 66°) → Blue: (58.716, 11.743, 112.5°)
    private final Pose finalShootPose = new Pose(58.716, 11.743, Math.toRadians(112.5));
    // Red: (105.14, 12.094, 90°) → Blue: (38.86, 12.094, 90°)
    private final Pose parkPose = new Pose(38.86, 12.094064949608057, Math.toRadians(90));

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
        // Blue: 90° -> 112.5°
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112.5))
                .build();

        // Path 2: Shoot Pose -> Sample 1 (BezierLine)
        // Blue: 112.5° -> 180°
        path2_toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, sample1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(180))
                .build();

        // Path 3: Sample 1 -> Push Zone (BezierLine)
        // Blue: 180° -> 180°
        path3_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, pushZone))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: Push Zone -> Sample 2 (BezierLine)
        // Blue: 180° -> 180°
        path4_toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone, sample2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 5: Sample 2 -> Shoot Pose (BezierLine)
        // Blue: 180° -> 112.5°
        path5_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, finalShootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112.5))
                .build();

        // Path 6: Shoot Pose -> Park (BezierLine)
        // Blue: 112.5° -> 90°
        path6_toPark = follower.pathBuilder()
                .addPath(new BezierLine(finalShootPose, parkPose))
                .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(90))
                .build();
    }
}
