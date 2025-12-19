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
 * Autonomous OpMode for the Red Alliance (Near Side).
 * Mirrored from BlueNear about X=72 axis.
 * 
 * Mirror formula:
 *   newX = 144 - oldX
 *   newY = oldY
 *   newHeading = 180° - oldHeading
 */
@Autonomous(name = "Red Near Auto", group = "Red")
public class RedNear extends AutoCommandBase {

    // Path chains
    private PathChain path1_toShootPose;
    private PathChain path2_pickupSample1;  // BezierCurve
    private PathChain path3_toShootPose;
    private PathChain path4_pickupSample2;  // BezierCurve
    private PathChain path5_toShootPose;
    private PathChain path6_pickupSample3;  // BezierCurve
    private PathChain path7_toShootPose;

    // Poses mirrored from Blue (X=72 axis)
    // Blue: (25.509, 129.474, 144°) → Red: (118.491, 129.474, 36°)
    private final Pose startPose = new Pose(118.491, 129.474, Math.toRadians(36));
    // Blue: (37.200, 110.342, 135°) → Red: (106.800, 110.342, 45°)
    private final Pose shootPose = new Pose(106.800, 110.342, Math.toRadians(45));
    // Blue: (24.269, 83.594, 180°) → Red: (119.731, 83.594, 0°)
    private final Pose sample1Pose = new Pose(119.731, 83.594, Math.toRadians(0));
    // Blue: (24.446, 60.033, 180°) → Red: (119.554, 60.033, 0°)
    private final Pose sample2Pose = new Pose(119.554, 60.033, Math.toRadians(0));
    // Blue: (24.446, 35.587, 180°) → Red: (119.554, 35.587, 0°)
    private final Pose sample3Pose = new Pose(119.554, 35.587, Math.toRadians(0));

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
        // Shoot pose for Path 1 (mirrored)
        // Blue: (37.200, 110.520, 135°) → Red: (106.800, 110.520, 45°)
        Pose shootPose1 = new Pose(106.800, 110.520, Math.toRadians(45));

        // Path 1: Start -> Shoot Pose (BezierLine)
        // Blue: 144° -> 135° → Red: 36° -> 45°
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        // Path 2: Shoot Pose -> Sample 1 (BezierCurve, 1 control point)
        // Blue control: (81.664, 80.936) → Red: (62.336, 80.936)
        // Blue: 135° -> 180° → Red: 45° -> 0°
        path2_pickupSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose1,
                        new Pose(62.336, 80.936, 0),
                        sample1Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 3: Sample 1 -> Shoot Pose (BezierLine)
        // Blue: 180° -> 135° → Red: 0° -> 45°
        path3_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Path 4: Shoot Pose -> Sample 2 (BezierCurve, 2 control points)
        // Blue controls: (59.344, 99.182), (70.149, 56.668)
        // → Red: (84.656, 99.182), (73.851, 56.668)
        // Blue: 135° -> 180° → Red: 45° -> 0°
        path4_pickupSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(84.656, 99.182, 0),
                        new Pose(73.851, 56.668, 0),
                        sample2Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 5: Sample 2 -> Shoot Pose (BezierLine)
        // Blue: 180° -> 135° → Red: 0° -> 45°
        path5_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Path 6: Shoot Pose -> Sample 3 (BezierCurve, 3 control points)
        // Blue controls: (61.646, 98.828), (52.081, 62.691), (68.555, 34.524)
        // → Red: (82.354, 98.828), (91.919, 62.691), (75.445, 34.524)
        // Blue: 135° -> 180° → Red: 45° -> 0°
        path6_pickupSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose,
                        new Pose(82.354, 98.828, 0),
                        new Pose(91.919, 62.691, 0),
                        new Pose(75.445, 34.524, 0),
                        sample3Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 7: Sample 3 -> Shoot Pose (BezierLine)
        // Blue: 180° -> 135° → Red: 0° -> 45°
        path7_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample3Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }
}
