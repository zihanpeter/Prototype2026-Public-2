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
 * Custom version: Copy of Multi-Gate for tuning.
 * 10 paths total.
 */
@Autonomous(name = "Blue Near Two", group = "Near")
public class BlueNearTwo extends AutoCommandBase {

    // Path chains (10 paths - Sample 3 goes directly to Shoot, added parking)
    private PathChain path1_toShootPose;
    private PathChain path2_toSample1;
    private PathChain path3_toGate;
    private PathChain path4_toShootPose;
    private PathChain path5_toSample2;
    private PathChain path6_toGate;
    private PathChain path7_toShootPose;
    private PathChain path8_toSample3;
    private PathChain path9_toShootPose;
    private PathChain path10_toParking;

    // Poses from JSON (updated - gate heading changed to 90°)
    private final Pose startPose = new Pose(25.509, 129.474, Math.toRadians(144));
    private final Pose shootPose1 = new Pose(35.229, 112.0, Math.toRadians(135));
    private final Pose sample1Pose = new Pose(20.318, 84.175, Math.toRadians(180));
    private final Pose gatePose = new Pose(14, 76.434, Math.toRadians(90));
    private final Pose shootPose2 = new Pose(35.315, 111.910, Math.toRadians(135));
    private final Pose sample2Pose = new Pose(21.769, 59.664, Math.toRadians(180));
    private final Pose sample3Pose = new Pose(21.447, 35.637, Math.toRadians(180));
    private final Pose shootPose3 = new Pose(35.476, 111.910, Math.toRadians(135));
    private final Pose parkingPose = new Pose(18.060, 95.946, Math.toRadians(180));

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
                // 9. Path 9: Sample 3 -> Shoot Pose (Line, final)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path9_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(1300),

                // =========================================================
                // 10. Path 10: Shoot Pose -> Parking
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),
                new AutoDriveCommand(follower, path10_toParking),

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
        // startDeg=144 -> endDeg=135
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                .build();

        // Path 2: Shoot Pose 1 -> Sample 1 (BezierCurve, 2 control points)
        // startDeg=135 -> endDeg=180
        path2_toSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose1,
                        new Pose(59.180, 112.394, 0),
                        new Pose(75.628, 77.080, 0),
                        sample1Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 3: Sample 1 -> Gate (BezierCurve, 1 control point)
        // startDeg=180 -> endDeg=90
        path3_toGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample1Pose,
                        new Pose(37.733, 74.661, 0),
                        gatePose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        // Path 4: Gate -> Shoot Pose 2 (BezierLine)
        // startDeg=90 -> endDeg=135
        path4_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        // Path 5: Shoot Pose 2 -> Sample 2 (BezierCurve, 3 control points)
        // startDeg=135 -> endDeg=180
        path5_toSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose2,
                        new Pose(53.859, 114.813, 0),
                        new Pose(70.952, 82.562, 0),
                        new Pose(58.213, 53.536, 0),
                        sample2Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 6: Sample 2 -> Gate (BezierCurve, 1 control point)
        // startDeg=180 -> endDeg=90
        path6_toGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample2Pose,
                        new Pose(44.184, 70.146, 0),
                        gatePose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        // Path 7: Gate -> Shoot Pose 2 (BezierLine)
        // startDeg=90 -> endDeg=135
        path7_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        // Path 8: Shoot Pose 2 -> Sample 3 (BezierCurve, 3 control points)
        // startDeg=135 -> endDeg=180
        path8_toSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose2,
                        new Pose(85.787, 107.395, 0),
                        new Pose(44.022, 53.375, 0),
                        new Pose(66.759, 31.283, 0),
                        sample3Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 9: Sample 3 -> Shoot Pose 1 (BezierLine, direct)
        // startDeg=180 -> endDeg=135
        path9_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(sample3Pose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // Path 10: Shoot Pose 1 -> Parking (BezierLine)
        // startDeg=135 -> endDeg=180
        path10_toParking = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, parkingPose))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();
    }
}

