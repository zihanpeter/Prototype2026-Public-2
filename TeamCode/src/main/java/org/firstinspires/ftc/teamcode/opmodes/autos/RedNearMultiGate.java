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
 * Multi-Gate version: Shoots at intermediate position after each sample pickup.
 * Mirrored from BlueNearMultiGate about X=72 axis.
 * 10 paths total.
 * 
 * Mirror formula:
 *   newX = 144 - oldX
 *   newY = oldY
 *   newHeading = 180° - oldHeading
 */
@Autonomous(name = "Red Near Multi-Gate", group = "Red")
public class RedNearMultiGate extends AutoCommandBase {

    // Path chains (10 paths)
    private PathChain path1_toShootPose;
    private PathChain path2_toSample1;
    private PathChain path3_toGate;
    private PathChain path4_toShootPose;
    private PathChain path5_toSample2;
    private PathChain path6_toGate;
    private PathChain path7_toShootPose;
    private PathChain path8_toSample3;
    private PathChain path9_toGate;
    private PathChain path10_toShootPose;

    // Poses mirrored from Blue (X=72 axis)
    // Blue: (25.509, 129.474, 144°) → Red: (118.491, 129.474, 36°)
    private final Pose startPose = new Pose(118.491, 129.474, Math.toRadians(36));
    // Blue: (35.229, 112.0, 135°) → Red: (108.771, 112.0, 45°)
    private final Pose shootPose1 = new Pose(108.771, 112.0, Math.toRadians(45));
    // Blue: (20.318, 84.175, 180°) → Red: (123.682, 84.175, 0°)
    private final Pose sample1Pose = new Pose(123.682, 84.175, Math.toRadians(0));
    // Blue: (15.964, 72.887, 90°) → Red: (128.036, 72.887, 90°)
    private final Pose gatePose = new Pose(128.036, 72.887, Math.toRadians(90));
    // Blue: (35.315, 111.910, 135°) → Red: (108.685, 111.910, 45°)
    private final Pose shootPose2 = new Pose(108.685, 111.910, Math.toRadians(45));
    // Blue: (21.769, 59.664, 180°) → Red: (122.231, 59.664, 0°)
    private final Pose sample2Pose = new Pose(122.231, 59.664, Math.toRadians(0));
    // Blue: (21.447, 35.637, 180°) → Red: (122.553, 35.637, 0°)
    private final Pose sample3Pose = new Pose(122.553, 35.637, Math.toRadians(0));
    // Blue: (35.476, 111.910, 135°) → Red: (108.524, 111.910, 45°)
    private final Pose shootPose3 = new Pose(108.524, 111.910, Math.toRadians(45));

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        return new SequentialCommandGroup(
                // =========================================================
                // Initialize: Start intake at 0.65 power
                // =========================================================
                new InstantCommand(() -> intake.startIntake()),

                // =========================================================
                // 1. Path 1: Start -> Shoot Pose (Preload)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path1_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 2. Path 2: Shoot Pose -> Sample 1 (Curve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path2_toSample1),

                // =========================================================
                // 3. Path 3: Sample 1 -> Gate (Curve)
                // =========================================================
                new AutoDriveCommand(follower, path3_toGate),

                // =========================================================
                // 4. Path 4: Gate -> Shoot Pose (Line)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path4_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 5. Path 5: Shoot Pose -> Sample 2 (Curve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path5_toSample2),

                // =========================================================
                // 6. Path 6: Sample 2 -> Gate (Curve)
                // =========================================================
                new AutoDriveCommand(follower, path6_toGate),

                // =========================================================
                // 7. Path 7: Gate -> Shoot Pose (Line)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path7_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 8. Path 8: Shoot Pose -> Sample 3 (Curve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path8_toSample3),

                // =========================================================
                // 9. Path 9: Sample 3 -> Gate (Curve)
                // =========================================================
                new AutoDriveCommand(follower, path9_toGate),

                // =========================================================
                // 10. Path 10: Gate -> Shoot Pose (Line, final)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path10_toShootPose),
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
        // Path 1: Start -> Shoot Pose 1 (BezierLine)
        // Blue: 144°->135° → Red: 36°->45°
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        // Path 2: Shoot Pose 1 -> Sample 1 (BezierCurve, 2 control points)
        // Blue: 135°->180° → Red: 45°->0°
        // Blue ctrl: (59.180, 112.394), (75.628, 77.080) → Red: (84.820, 112.394), (68.372, 77.080)
        path2_toSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose1,
                        new Pose(84.820, 112.394, 0),
                        new Pose(68.372, 77.080, 0),
                        sample1Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 3: Sample 1 -> Gate (BezierCurve, 1 control point)
        // Blue: 180°->90° → Red: 0°->90°
        // Blue ctrl: (37.733, 74.661) → Red: (106.267, 74.661)
        path3_toGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample1Pose,
                        new Pose(106.267, 74.661, 0),
                        gatePose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Path 4: Gate -> Shoot Pose 2 (BezierLine)
        // Blue: 90°->135° → Red: 90°->45°
        path4_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        // Path 5: Shoot Pose 2 -> Sample 2 (BezierCurve, 3 control points)
        // Blue: 135°->180° → Red: 45°->0°
        // Blue ctrl: (53.859, 114.813), (70.952, 82.562), (58.213, 53.536)
        // → Red: (90.141, 114.813), (73.048, 82.562), (85.787, 53.536)
        path5_toSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose2,
                        new Pose(90.141, 114.813, 0),
                        new Pose(73.048, 82.562, 0),
                        new Pose(85.787, 53.536, 0),
                        sample2Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 6: Sample 2 -> Gate (BezierCurve, 1 control point)
        // Blue: 180°->90° → Red: 0°->90°
        // Blue ctrl: (44.184, 70.146) → Red: (99.816, 70.146)
        path6_toGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample2Pose,
                        new Pose(99.816, 70.146, 0),
                        gatePose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Path 7: Gate -> Shoot Pose 2 (BezierLine)
        // Blue: 90°->135° → Red: 90°->45°
        path7_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        // Path 8: Shoot Pose 2 -> Sample 3 (BezierCurve, 3 control points)
        // Blue: 135°->180° → Red: 45°->0°
        // Blue ctrl: (85.787, 107.395), (44.022, 53.375), (66.759, 31.283)
        // → Red: (58.213, 107.395), (99.978, 53.375), (77.241, 31.283)
        path8_toSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose2,
                        new Pose(58.213, 107.395, 0),
                        new Pose(99.978, 53.375, 0),
                        new Pose(77.241, 31.283, 0),
                        sample3Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 9: Sample 3 -> Gate (BezierCurve, 1 control point)
        // Blue: 180°->90° → Red: 0°->90°
        // Blue ctrl: (54.181, 74.983) → Red: (89.819, 74.983)
        path9_toGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample3Pose,
                        new Pose(89.819, 74.983, 0),
                        gatePose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Path 10: Gate -> Shoot Pose 3 (BezierLine)
        // Blue: 90°->135° → Red: 90°->45°
        path10_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose3))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();
    }
}

