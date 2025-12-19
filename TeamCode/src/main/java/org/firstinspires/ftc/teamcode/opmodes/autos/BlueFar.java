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
 * Autonomous OpMode for the Blue Alliance (Far Side).
 * 7 paths, all BezierLine (straight lines).
 */
@Autonomous(name = "Blue Far Auto", group = "Blue")
public class BlueFar extends AutoCommandBase {

    // Path chains (7 paths)
    private PathChain path1_toShootPose;
    private PathChain path2_toSample1;
    private PathChain path3_toPushZone;
    private PathChain path4_toSample2;
    private PathChain path5_toPushZone;
    private PathChain path6_toSample3;
    private PathChain path7_toShootPose;

    // Poses from JSON
    private final Pose startPose = new Pose(56.807, 8.073, Math.toRadians(90));
    private final Pose shootPose = new Pose(61.358, 18.495, Math.toRadians(120));
    private final Pose sample1Pose = new Pose(8.367, 10.275, Math.toRadians(180));
    private final Pose pushZone1 = new Pose(30.679, 10.569, Math.toRadians(180));
    private final Pose sample2Pose = new Pose(8.367, 10.275, Math.toRadians(180));
    private final Pose pushZone2 = new Pose(30.826, 10.569, Math.toRadians(180));
    private final Pose sample3Pose = new Pose(8.514, 10.275, Math.toRadians(180));
    private final Pose finalShootPose = new Pose(61.358, 18.495, Math.toRadians(116));

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
        // startDeg=90 -> endDeg=120
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();

        // Path 2: Shoot Pose -> Sample 1 (BezierLine)
        // startDeg=120 -> endDeg=180
        path2_toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, sample1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .build();

        // Path 3: Sample 1 -> Push Zone (BezierLine)
        // startDeg=180 -> endDeg=180
        path3_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, pushZone1))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: Push Zone -> Sample 2 (BezierLine)
        // startDeg=180 -> endDeg=180
        path4_toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone1, sample2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 5: Sample 2 -> Push Zone (BezierLine)
        // startDeg=180 -> endDeg=180
        path5_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, pushZone2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 6: Push Zone -> Sample 3 (BezierLine)
        // startDeg=180 -> endDeg=180
        path6_toSample3 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone2, sample3Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 7: Sample 3 -> Shoot Pose (BezierLine)
        // startDeg=180 -> endDeg=116
        path7_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample3Pose, finalShootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(116))
                .build();
    }
}

