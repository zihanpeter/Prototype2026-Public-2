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
 * Updated with new 8-path JSON data.
 */
@Autonomous(name = "Blue Near Auto", group = "Blue")
public class BlueNear extends AutoCommandBase {

    // Path chains (8 paths now)
    private PathChain path1_toShootPose;
    private PathChain path2_pickupSample1;
    private PathChain path3_toIntermediate;
    private PathChain path4_toShootPose;
    private PathChain path5_pickupSample2;
    private PathChain path6_toShootPose;
    private PathChain path7_pickupSample3;
    private PathChain path8_toShootPose;

    // Poses from new JSON
    private final Pose startPose = new Pose(25.509, 129.474, Math.toRadians(144));
    private final Pose shootPose1 = new Pose(35.229, 112.0, Math.toRadians(135));
    private final Pose sample1Pose = new Pose(18.936, 83.670, Math.toRadians(180));
    private final Pose intermediatePose = new Pose(14.826, 70.312, Math.toRadians(0));
    private final Pose shootPose2 = new Pose(35.376, 111.706, Math.toRadians(135));
    private final Pose sample2Pose = new Pose(18.936, 59.890, Math.toRadians(180));
    private final Pose shootPose3 = new Pose(35.376, 111.853, Math.toRadians(135));
    private final Pose sample3Pose = new Pose(18.936, 35.376, Math.toRadians(180));

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        return new SequentialCommandGroup(
                // =========================================================
                // Initialize: Start intake at 0.5 power
                // =========================================================
                new InstantCommand(() -> intake.startIntake()),

                // =========================================================
                // 1. Path 1: Start -> Shoot Pose 1 (Preload)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW)),
                new AutoDriveCommand(follower, path1_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(800),

                // =========================================================
                // 2. Path 2: Shoot Pose 1 -> Sample 1 (BezierCurve) - FULL POWER
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFullPower(true);
                }),
                new AutoDriveCommand(follower, path2_pickupSample1),

                // =========================================================
                // 3. Path 3: Sample 1 -> Intermediate (BezierCurve) - FULL POWER
                // =========================================================
                new AutoDriveCommand(follower, path3_toIntermediate),

                // =========================================================
                // 4. Path 4: Intermediate -> Shoot Pose 2 (BezierLine)
                // =========================================================
                new InstantCommand(() -> {
                    intake.setFullPower(false);
                    shooter.setShooterState(Shooter.ShooterState.SLOW);
                }),
                new AutoDriveCommand(follower, path4_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(800),

                // =========================================================
                // 5. Path 5: Shoot Pose 2 -> Sample 2 (BezierCurve) - FULL POWER
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFullPower(true);
                }),
                new AutoDriveCommand(follower, path5_pickupSample2),

                // =========================================================
                // 6. Path 6: Sample 2 -> Shoot Pose 3 (BezierLine)
                // =========================================================
                new InstantCommand(() -> {
                    intake.setFullPower(false);
                    shooter.setShooterState(Shooter.ShooterState.SLOW);
                }),
                new AutoDriveCommand(follower, path6_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(800),

                // =========================================================
                // 7. Path 7: Shoot Pose 3 -> Sample 3 (BezierCurve) - FULL POWER
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFullPower(true);
                }),
                new AutoDriveCommand(follower, path7_pickupSample3),

                // =========================================================
                // 8. Path 8: Sample 3 -> Shoot Pose 3 (Final)
                // =========================================================
                new InstantCommand(() -> {
                    intake.setFullPower(false);
                    shooter.setShooterState(Shooter.ShooterState.SLOW);
                }),
                new AutoDriveCommand(follower, path8_toShootPose),
                new TransitCommand(transit, shooter).withTimeout(1500),

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

        // Path 2: Shoot Pose 1 -> Sample 1 (BezierCurve, 1 control point)
        // startDeg=135 -> endDeg=180, ctrl=(81.664, 80.936)
        path2_pickupSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose1,
                        new Pose(81.664, 80.936, 0),
                        sample1Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 3: Sample 1 -> Intermediate (BezierCurve, 1 control point)
        // startDeg=180 -> endDeg=0, ctrl=(55.927, 92.183)
        path3_toIntermediate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample1Pose,
                        new Pose(55.927, 92.183, 0),
                        intermediatePose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        // Path 4: Intermediate -> Shoot Pose 2 (BezierLine)
        // startDeg=0 -> endDeg=135
        path4_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(intermediatePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        // Path 5: Shoot Pose 2 -> Sample 2 (BezierCurve, 3 control points)
        // startDeg=135 -> endDeg=180
        // ctrls: (46.972, 106.862), (76.330, 85.725), (69.138, 53.431)
        path5_pickupSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose2,
                        new Pose(46.972, 106.862, 0),
                        new Pose(76.330, 85.725, 0),
                        new Pose(69.138, 53.431, 0),
                        sample2Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 6: Sample 2 -> Shoot Pose 3 (BezierCurve, 1 control point)
        // startDeg=180 -> endDeg=135, ctrl=(55.046, 59.743)
        path6_toShootPose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample2Pose,
                        new Pose(55.046, 59.743, 0),
                        shootPose3
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        // Path 7: Shoot Pose 3 -> Sample 3 (BezierCurve, 3 control points)
        // startDeg=135 -> endDeg=180
        // ctrls: (50.789, 103.193), (66.495, 31.413), (71.927, 33.321)
        path7_pickupSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose3,
                        new Pose(50.789, 103.193, 0),
                        new Pose(66.495, 31.413, 0),
                        new Pose(71.927, 33.321, 0),
                        sample3Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        // Path 8: Sample 3 -> Shoot Pose 3 (BezierCurve, 1 control point)
        // startDeg=180 -> endDeg=135, ctrl=(50.642, 81.028)
        path8_toShootPose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        sample3Pose,
                        new Pose(50.642, 81.028, 0),
                        shootPose3
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }
}
