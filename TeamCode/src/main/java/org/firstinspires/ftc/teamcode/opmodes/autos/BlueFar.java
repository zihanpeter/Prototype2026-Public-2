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
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Autonomous OpMode for the Blue Alliance (Far Side).
 * 6 paths (2 push cycles), all BezierLine (straight lines).
 */
@Autonomous(name = "Blue Far Auto", group = "Blue")
public class BlueFar extends AutoCommandBase {

    // Path chains (5 paths - 1 push cycle per loop)
    private PathChain path1_toShootPose;
    private PathChain path2_toSample1;
    private PathChain path3_toPushZone;
    private PathChain path4_toSample2;
    private PathChain path5_toShootPose;

    // Poses from new JSON
    private final Pose startPose = new Pose(55.269, 7.953, Math.toRadians(90));
    private final Pose shootPose = new Pose(58.716, 11.890, Math.toRadians(114));
    private final Pose sample1Pose = new Pose(11.303, 10.569, Math.toRadians(180));
    private final Pose pushZone = new Pose(30.679, 10.569, Math.toRadians(180));
    private final Pose sample2Pose = new Pose(11.450, 10.569, Math.toRadians(180));
    private final Pose finalShootPose = new Pose(58.716, 11.743, Math.toRadians(114));

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
                        new TransitCommand(transit, shooter).withTimeout(1300),
                        new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
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
                new TransitCommand(transit, shooter).withTimeout(1300),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),

                // =========================================================
                // 2-6. Infinite Loop: 往返2次 + 射击
                // =========================================================
                pushAndShootLoop
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Shoot Pose (BezierLine)
        // startDeg=90 -> endDeg=114
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        // Path 2: Shoot Pose -> Sample 1 (BezierLine)
        // startDeg=114 -> endDeg=180
        path2_toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, sample1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        // Path 3: Sample 1 -> Push Zone (BezierLine)
        // startDeg=180 -> endDeg=180
        path3_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, pushZone))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: Push Zone -> Sample 2 (BezierLine)
        // startDeg=180 -> endDeg=180
        path4_toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone, sample2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 5: Sample 2 -> Shoot Pose (BezierLine)
        // startDeg=180 -> endDeg=114
        path5_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, finalShootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();
    }
}
