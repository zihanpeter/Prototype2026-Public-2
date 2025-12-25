package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoTransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.HoldPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Autonomous OpMode for the Red Alliance (Far Side) - Shanghai Version.
 * Mirrored from BlueFarShanghai (X=72 axis).
 * 7 paths total, with special intake behavior on Path 5.
 */
@Autonomous(name = "Red Far Shanghai", group = "Red")
public class RedFarShanghai extends AutoCommandBase {

    // Path chains
    private PathChain path1_toShootPose;
    private PathChain path2_toSample;
    private PathChain path3_toShootPose;
    private PathChain path4_toPushStart;
    private PathChain path5_toPushEnd;
    private PathChain path6_toShootPose;
    private PathChain path7_toParking;

    // Poses mirrored from Blue (X=72 axis): newX = 144 - oldX, newHeading = 180 - oldHeading
    // Blue: (55.27, 7.95, 90°) → Red: (88.73, 7.95, 90°)
    private final Pose startPose = new Pose(88.73, 7.95, Math.toRadians(90));
    // Blue: (58.72, 11.89, 114°) → Red: (85.28, 11.89, 66°)
    private final Pose shootPose1 = new Pose(85.28, 11.89, Math.toRadians(66));
    // Blue: (13.71, 34.19, 180°) → Red: (130.29, 34.19, 0°)
    private final Pose samplePose = new Pose(130.29, 34.19, Math.toRadians(0));
    // Blue: (58.72, 11.89, 114°) → Red: (85.28, 11.89, 66°)
    private final Pose shootPose2 = new Pose(85.28, 11.89, Math.toRadians(66));
    // Blue: (7.740, 25.317, 270°) → Red: (136.260, 25.317, 270°)
    private final Pose pushStartPose = new Pose(136.260, 25.317, Math.toRadians(270));
    // Blue: (8.063, 9.837, 270°) → Red: (135.937, 9.837, 270°)
    private final Pose pushEndPose = new Pose(135.937, 9.837, Math.toRadians(270));
    // Blue: (58.72, 11.89, 123°) → Red: (85.28, 11.89, 57°)
    private final Pose shootPose3 = new Pose(85.28, 11.89, Math.toRadians(57));
    // Blue: (38.862, 12.094, 90°) → Red: (105.138, 12.094, 90°)
    private final Pose parkingPose = new Pose(105.138, 12.094, Math.toRadians(90));

    // Control points for Path 2 (BezierCurve) - mirrored
    // Blue: (57.245, 37.088) → Red: (86.755, 37.088)
    private final Pose ctrl1 = new Pose(86.755, 37.088, 0);
    // Blue: (61.76, 27.90) → Red: (82.24, 27.90)
    private final Pose ctrl2 = new Pose(82.24, 27.90, 0);
    // Blue: (65.31, 37.09) → Red: (78.69, 37.09)
    private final Pose ctrl3 = new Pose(78.69, 37.09, 0);
    
    // Control point for Path 6 (BezierCurve) - mirrored
    // Blue: (35.153, 32.573) → Red: (108.847, 32.573)
    private final Pose ctrl6 = new Pose(108.847, 32.573, 0);

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
                // Path 1: Start -> Shoot Pose 1 (Preload)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new AutoDriveCommand(follower, path1_toShootPose),
                new ParallelDeadlineGroup(
                        new AutoTransitCommand(transit, shooter, follower, shootPose1, intake).withTimeout(3000),
                        new HoldPositionCommand(follower)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),

                // =========================================================
                // Path 2: Shoot Pose -> Sample (Curve) - Intake at 0.65 power
                // =========================================================
                new InstantCommand(() -> intake.setFullPower(true)),  // 0.65 power
                new AutoDriveCommand(follower, path2_toSample),
                new InstantCommand(() -> intake.setFullPower(false)), // Back to normal

                // =========================================================
                // Path 3: Sample -> Shoot Pose 2
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new AutoDriveCommand(follower, path3_toShootPose),
                new ParallelDeadlineGroup(
                        new AutoTransitCommand(transit, shooter, follower, shootPose2, intake).withTimeout(1300),
                        new HoldPositionCommand(follower)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),

                // =========================================================
                // Path 4: Shoot Pose -> Push Start
                // =========================================================
                new AutoDriveCommand(follower, path4_toPushStart),

                // =========================================================
                // Path 5: Push Start -> Push End (Intake at 0.65 power)
                // =========================================================
                new InstantCommand(() -> intake.setFullPower(true)),  // 0.65 power
                new AutoDriveCommand(follower, path5_toPushEnd),
                new InstantCommand(() -> intake.setFullPower(false)), // Back to normal

                // =========================================================
                // Path 6: Push End -> Shoot Pose 3 (No auto-aim, rely on path endpoint)
                // =========================================================
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST)),
                new AutoDriveCommand(follower, path6_toShootPose),
                // Auto-aim disabled for now - relying on path endpoint accuracy
                new ParallelDeadlineGroup(
                        new AutoTransitCommand(transit, shooter, follower, shootPose3, intake).withTimeout(2000),
                        new HoldPositionCommand(follower)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),

                // =========================================================
                // Path 7: Shoot Pose -> Parking
                // =========================================================
                new AutoDriveCommand(follower, path7_toParking),

                // =========================================================
                // Finish
                // =========================================================
                new InstantCommand(() -> {
                    intake.stopIntake();
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                })
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Shoot Pose 1 (BezierLine)
        // Blue: 90° -> 114° → Red: 90° -> 66°
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))
                .build();

        // Path 2: Shoot Pose -> Sample (BezierCurve with 3 control points)
        // Blue: 114° -> 180° → Red: 66° -> 0°
        path2_toSample = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, ctrl1, ctrl2, ctrl3, samplePose))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .build();

        // Path 3: Sample -> Shoot Pose 2 (BezierLine)
        // Blue: 180° -> 114° → Red: 0° -> 66°
        path3_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(samplePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();

        // Path 4: Shoot Pose -> Push Start (BezierLine)
        // Blue: 114° -> 270° → Red: 66° -> 270°
        path4_toPushStart = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, pushStartPose))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(270))
                .build();

        // Path 5: Push Start -> Push End (BezierLine)
        // Blue: 270° -> 270° → Red: 270° -> 270°
        path5_toPushEnd = follower.pathBuilder()
                .addPath(new BezierLine(pushStartPose, pushEndPose))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        // Path 6: Push End -> Shoot Pose 3 (BezierCurve with 1 control point)
        // Blue: 270° -> 123° → Red: 270° -> 57°
        path6_toShootPose = follower.pathBuilder()
                .addPath(new BezierCurve(pushEndPose, ctrl6, shootPose3))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(57))
                .build();

        // Path 7: Shoot Pose -> Parking (BezierLine)
        // Blue: 123° -> 90° → Red: 57° -> 90°
        path7_toParking = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, parkingPose))
                .setLinearHeadingInterpolation(Math.toRadians(57), Math.toRadians(90))
                .build();
    }
}

