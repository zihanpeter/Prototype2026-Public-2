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
 * Updated with new 8-path JSON data.
 * 
 * Mirror formula:
 *   newX = 144 - oldX
 *   newY = oldY
 *   newHeading = 180° - oldHeading
 */
@Autonomous(name = "Red Near One", group = "Near")
public class RedNearOne extends AutoCommandBase {

    // Path chains (9 paths now - added parking)
    private PathChain path1_toShootPose;
    private PathChain path2_pickupSample1;
    private PathChain path3_toIntermediate;
    private PathChain path4_toShootPose;
    private PathChain path5_pickupSample2;
    private PathChain path6_toShootPose;
    private PathChain path7_pickupSample3;
    private PathChain path8_toShootPose;
    private PathChain path9_toParking;

    // Poses mirrored from BlueNear (X=72 axis): RedX = 144 - BlueX, RedHeading = 180° - BlueHeading
    // Blue: (25.509, 129.474, 144°) → Red: (118.491, 129.474, 36°)
    private final Pose startPose = new Pose(118.491, 129.474, Math.toRadians(36));
    // Blue: (35.229, 112.0, 135°) → Red: (108.771, 112.0, 45°)
    private final Pose shootPose1 = new Pose(108.771, 112.0, Math.toRadians(45));
    // Blue: (20.550, 83.817, 180°) → Red: (123.450, 83.817, 0°)
    private final Pose sample1Pose = new Pose(123.450, 83.817, Math.toRadians(0));
    // Blue: (14, 76.434, 90°) → Red: (130, 76.434, 90°)
    private final Pose intermediatePose = new Pose(130, 76.434, Math.toRadians(90));
    // Blue: (30.376, 111.706, 135°) → Red: (113.624, 111.706, 45°)
    private final Pose shootPose2 = new Pose(113.624, 111.706, Math.toRadians(45));
    // Blue: (20.550, 59.743, 180°) → Red: (123.450, 59.743, 0°)
    private final Pose sample2Pose = new Pose(123.450, 59.743, Math.toRadians(0));
    // Blue: (30.376, 111.853, 135°) → Red: (113.624, 111.853, 45°)
    private final Pose shootPose3 = new Pose(113.624, 111.853, Math.toRadians(45));
    // Blue: (16.120, 35.587, 180°) → Red: (127.880, 35.587, 0°)
    private final Pose sample3Pose = new Pose(127.880, 35.587, Math.toRadians(0));
    // Blue: (18.060, 95.946, 180°) → Red: (125.940, 95.946, 0°)
    private final Pose parkingPose = new Pose(125.940, 95.946, Math.toRadians(0));

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
                new InstantCommand(() -> {
                    intake.startIntake();
                    intake.setFullPower(true); // Use 0.65 power for Near auto
                }),

                // =========================================================
                // 1. Path 1: Start -> Shoot Pose 1 (Preload)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path1_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 2. Path 2: Shoot Pose 1 -> Sample 1 (BezierCurve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path2_pickupSample1),

                // =========================================================
                // 3. Path 3: Sample 1 -> Intermediate (BezierCurve)
                // =========================================================
                new AutoDriveCommand(follower, path3_toIntermediate),

                // =========================================================
                // 4. Path 4: Intermediate -> Shoot Pose 2 (BezierLine)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new InstantCommand(() -> intake.setFastShooting(true)),  // 0.75 power during approach
                new AutoDriveCommand(follower, path4_toShootPose),
                new InstantCommand(() -> intake.setFastShooting(false)), // Back to normal
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 5. Path 5: Shoot Pose 2 -> Sample 2 (BezierCurve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path5_pickupSample2),

                // =========================================================
                // 6. Path 6: Sample 2 -> Shoot Pose 3 (BezierCurve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new InstantCommand(() -> intake.setFastShooting(true)),  // 0.75 power during approach
                new AutoDriveCommand(follower, path6_toShootPose),
                new InstantCommand(() -> intake.setFastShooting(false)), // Back to normal
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 7. Path 7: Shoot Pose 3 -> Sample 3 (BezierCurve)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path7_pickupSample3),

                // =========================================================
                // 8. Path 8: Sample 3 -> Shoot Pose 3 (BezierCurve, Final)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new InstantCommand(() -> intake.setFastShooting(true)),  // 0.75 power during approach
                new AutoDriveCommand(follower, path8_toShootPose),
                new InstantCommand(() -> intake.setFastShooting(false)), // Back to normal
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 9. Path 9: Shoot Pose -> Parking
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path9_toParking),

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
        // Blue: 144° -> 135° → Red: 36° -> 45°
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                .build();

        // Path 2: Shoot Pose 1 -> Sample 1 (BezierCurve, 1 control point)
        // Blue ctrl: (81.664, 80.936) → Red: (62.336, 80.936)
        // Blue: 135° -> 180° → Red: 45° -> 0°
        path2_pickupSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose1,
                        new Pose(62.336, 80.936, 0),
                        sample1Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 3: Sample 1 -> Intermediate (BezierCurve, 1 control point)
        // Blue ctrl: (37.733, 74.661) → Red: (106.267, 74.661) - same as MultiGate
        // Blue: 180° -> 90° → Red: 0° -> 90°
        path3_toIntermediate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample1Pose,
                        new Pose(106.267, 74.661, 0),
                        intermediatePose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        // Path 4: Intermediate -> Shoot Pose 2 (BezierLine)
        // Blue: 90° -> 135° → Red: 90° -> 45°
        path4_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(intermediatePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        // Path 5: Shoot Pose 2 -> Sample 2 (BezierCurve, 3 control points)
        // Blue ctrls: (46.972, 106.862), (76.330, 85.725), (69.138, 53.431)
        // → Red: (97.028, 106.862), (67.670, 85.725), (74.862, 53.431)
        // Blue: 135° -> 180° → Red: 45° -> 0°
        path5_pickupSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose2,
                        new Pose(97.028, 106.862, 0),
                        new Pose(67.670, 85.725, 0),
                        new Pose(74.862, 53.431, 0),
                        sample2Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 6: Sample 2 -> Shoot Pose 3 (BezierCurve, 1 control point)
        // Blue ctrl: (55.046, 59.743) → Red: (88.954, 59.743)
        // Blue: 180° -> 135° → Red: 0° -> 45°
        path6_toShootPose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample2Pose,
                        new Pose(88.954, 59.743, 0),
                        shootPose3
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Path 7: Shoot Pose 3 -> Sample 3 (BezierCurve, 3 control points)
        // Blue ctrls: (50.789, 103.193), (66.495, 31.413), (71.927, 33.321)
        // → Red: (93.211, 103.193), (77.505, 31.413), (72.073, 33.321)
        // Blue: 135° -> 180° → Red: 45° -> 0°
        path7_pickupSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose3,
                        new Pose(93.211, 103.193, 0),
                        new Pose(77.505, 31.413, 0),
                        new Pose(72.073, 33.321, 0),
                        sample3Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        // Path 8: Sample 3 -> Shoot Pose 3 (BezierCurve, 1 control point)
        // Blue ctrl: (50.642, 81.028) → Red: (93.358, 81.028)
        // Blue: 180° -> 135° → Red: 0° -> 45°
        path8_toShootPose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample3Pose,
                        new Pose(93.358, 81.028, 0),
                        shootPose3
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        // Path 9: Shoot Pose 3 -> Parking (BezierLine)
        // Blue: 135° -> 180° → Red: 45° -> 0°
        path9_toParking = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, parkingPose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }
}
