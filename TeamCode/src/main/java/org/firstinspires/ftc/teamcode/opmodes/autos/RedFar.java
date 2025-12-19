package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Autonomous OpMode for the Red Alliance (Far Side).
 * Mirrored from BlueFar about X=72 axis.
 * 7 paths, all BezierLine (straight lines).
 * 
 * Mirror formula:
 *   newX = 144 - oldX
 *   newY = oldY
 *   newHeading = 180° - oldHeading
 */
@Autonomous(name = "Red Far Auto", group = "Red")
public class RedFar extends AutoCommandBase {

    // Path chains (7 paths)
    private PathChain path1_toShootPose;
    private PathChain path2_toSample1;
    private PathChain path3_toPushZone;
    private PathChain path4_toSample2;
    private PathChain path5_toPushZone;
    private PathChain path6_toSample3;
    private PathChain path7_toShootPose;

    // Poses mirrored from Blue (X=72 axis)
    // Blue: (56.807, 8.073, 90°) → Red: (87.193, 8.073, 90°)
    private final Pose startPose = new Pose(87.193, 8.073, Math.toRadians(90));
    // Blue: (61.358, 18.495, 120°) → Red: (82.642, 18.495, 60°)
    private final Pose shootPose = new Pose(82.642, 18.495, Math.toRadians(60));
    // Blue: (8.367, 10.275, 180°) → Red: (135.633, 10.275, 0°)
    private final Pose sample1Pose = new Pose(135.633, 10.275, Math.toRadians(0));
    // Blue: (30.679, 10.569, 180°) → Red: (113.321, 10.569, 0°)
    private final Pose pushZone1 = new Pose(113.321, 10.569, Math.toRadians(0));
    // Blue: (8.367, 10.275, 180°) → Red: (135.633, 10.275, 0°)
    private final Pose sample2Pose = new Pose(135.633, 10.275, Math.toRadians(0));
    // Blue: (30.826, 10.569, 180°) → Red: (113.174, 10.569, 0°)
    private final Pose pushZone2 = new Pose(113.174, 10.569, Math.toRadians(0));
    // Blue: (8.514, 10.275, 180°) → Red: (135.486, 10.275, 0°)
    private final Pose sample3Pose = new Pose(135.486, 10.275, Math.toRadians(0));
    // Blue: (61.358, 18.495, 116°) → Red: (82.642, 18.495, 64°)
    private final Pose finalShootPose = new Pose(82.642, 18.495, Math.toRadians(64));

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        return new SequentialCommandGroup(
                // =========================================================
                // Initialize: Start intake
                // =========================================================
                new InstantCommand(() -> intake.startIntake()),

                // =========================================================
                // 1. Path 1: Start -> Shoot Pose (Preload)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path1_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 2. Path 2: Shoot Pose -> Sample 1
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path2_toSample1),

                // =========================================================
                // 3. Path 3: Sample 1 -> Push Zone
                // =========================================================
                new AutoDriveCommand(follower, path3_toPushZone),

                // =========================================================
                // 4. Path 4: Push Zone -> Sample 2
                // =========================================================
                new AutoDriveCommand(follower, path4_toSample2),

                // =========================================================
                // 5. Path 5: Sample 2 -> Push Zone
                // =========================================================
                new AutoDriveCommand(follower, path5_toPushZone),

                // =========================================================
                // 6. Path 6: Push Zone -> Sample 3
                // =========================================================
                new AutoDriveCommand(follower, path6_toSample3),

                // =========================================================
                // 7. Path 7: Sample 3 -> Shoot Pose (Final)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path7_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // Finish
                // =========================================================
                new InstantCommand(() -> {
                    intake.stopIntake();
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new InstantCommand(() -> telemetry.addData("Status", "Auto Completed"))
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Shoot Pose (BezierLine)
        // Blue: 90° -> 120° → Red: 90° -> 60°
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                .build();

        // Path 2: Shoot Pose -> Sample 1 (BezierLine)
        // Blue: 120° -> 180° → Red: 60° -> 0°
        path2_toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, sample1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                .build();

        // Path 3: Sample 1 -> Push Zone (BezierLine)
        // Blue: 180° -> 180° → Red: 0° -> 0°
        path3_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, pushZone1))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 4: Push Zone -> Sample 2 (BezierLine)
        // Blue: 180° -> 180° → Red: 0° -> 0°
        path4_toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone1, sample2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 5: Sample 2 -> Push Zone (BezierLine)
        // Blue: 180° -> 180° → Red: 0° -> 0°
        path5_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, pushZone2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 6: Push Zone -> Sample 3 (BezierLine)
        // Blue: 180° -> 180° → Red: 0° -> 0°
        path6_toSample3 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone2, sample3Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 7: Sample 3 -> Shoot Pose (BezierLine)
        // Blue: 180° -> 116° → Red: 0° -> 64°
        path7_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample3Pose, finalShootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(64))
                .build();
    }
}

