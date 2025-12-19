package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.BezierCurve;
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
 * Autonomous OpMode for the Blue Alliance (Near Side).
 * Updated with new JSON path data including BezierCurves.
 */
@Autonomous(name = "Blue Near Auto", group = "Blue")
public class BlueNear extends AutoCommandBase {

    // Path chains
    private PathChain path1_toShootPose;
    private PathChain path2_pickupSample1;  // BezierCurve
    private PathChain path3_toShootPose;
    private PathChain path4_pickupSample2;  // BezierCurve
    private PathChain path5_toShootPose;
    private PathChain path6_pickupSample3;  // BezierCurve
    private PathChain path7_toShootPose;

    // Poses from JSON
    private final Pose startPose = new Pose(25.509, 129.474, Math.toRadians(144));
    private final Pose shootPose = new Pose(37.200, 110.342, Math.toRadians(135));
    private final Pose sample1Pose = new Pose(24.269, 83.594, Math.toRadians(180));
    private final Pose sample2Pose = new Pose(24.446, 60.033, Math.toRadians(180));
    private final Pose sample3Pose = new Pose(24.446, 35.587, Math.toRadians(180));

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        return new SequentialCommandGroup(
                // =========================================================
                // 1. Path 1: Start -> Shoot Pose (Preload)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.MID)),
                new AutoDriveCommand(follower, path1_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(800),

                // =========================================================
                // 2. Path 2: Shoot Pose -> Sample 1 (BezierCurve)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.startIntake();
                }),
                new AutoDriveCommand(follower, path2_pickupSample1),

                // =========================================================
                // 3. Path 3: Sample 1 -> Shoot Pose
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.MID)),
                new AutoDriveCommand(follower, path3_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(800),

                // =========================================================
                // 4. Path 4: Shoot Pose -> Sample 2 (BezierCurve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path4_pickupSample2),

                // =========================================================
                // 5. Path 5: Sample 2 -> Shoot Pose
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.MID)),
                new AutoDriveCommand(follower, path5_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(800),

                // =========================================================
                // 6. Path 6: Shoot Pose -> Sample 3 (BezierCurve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path6_pickupSample3),

                // =========================================================
                // 7. Path 7: Sample 3 -> Shoot Pose (Final)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.MID)),
                new AutoDriveCommand(follower, path7_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(800),

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
        // Shoot pose for Path 1 (slightly different Y in JSON, using 110.520 for first approach)
        Pose shootPose1 = new Pose(37.200, 110.520, Math.toRadians(135));

        // Path 1: Start -> Shoot Pose (BezierLine, no control points)
        // JSON: startDeg=144 -> endDeg=135
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                .build();

        // Path 2: Shoot Pose -> Sample 1 (BezierCurve, 1 control point)
        // JSON: startDeg=135 -> endDeg=180, controlPoint=(81.664, 80.936)
        path2_pickupSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose1,
                        new Pose(81.664, 80.936, 0),
                        sample1Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 3: Sample 1 -> Shoot Pose (BezierLine, no control points)
        // JSON: startDeg=180 -> endDeg=135
        path3_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // Path 4: Shoot Pose -> Sample 2 (BezierCurve, 2 control points)
        // JSON: startDeg=135 -> endDeg=180
        // controlPoints: (59.344, 99.182), (70.149, 56.668)
        path4_pickupSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(59.344, 99.182, 0),
                        new Pose(70.149, 56.668, 0),
                        sample2Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 5: Sample 2 -> Shoot Pose (BezierLine, no control points)
        // JSON: startDeg=180 -> endDeg=135
        path5_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // Path 6: Shoot Pose -> Sample 3 (BezierCurve, 3 control points)
        // JSON: startDeg=135 -> endDeg=180
        // controlPoints: (61.646, 98.828), (52.081, 62.691), (68.555, 34.524)
        path6_pickupSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(61.646, 98.828, 0),
                        new Pose(52.081, 62.691, 0),
                        new Pose(68.555, 34.524, 0),
                        sample3Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 7: Sample 3 -> Shoot Pose (BezierLine, no control points)
        // JSON: startDeg=180 -> endDeg=135
        path7_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample3Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }
}
