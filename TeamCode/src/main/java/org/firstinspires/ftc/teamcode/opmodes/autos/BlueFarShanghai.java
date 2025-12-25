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
 * Autonomous OpMode for the Blue Alliance (Far Side) - Shanghai Version.
 * 7 paths total, with special intake behavior on Path 5.
 */
@Autonomous(name = "Blue Far Shanghai", group = "Blue")
public class BlueFarShanghai extends AutoCommandBase {

    // Path chains
    private PathChain path1_toShootPose;
    private PathChain path2_toSample;
    private PathChain path3_toShootPose;
    private PathChain path4_toPushStart;
    private PathChain path5_toPushEnd;
    private PathChain path6_toShootPose;
    private PathChain path7_toParking;

    // Poses from JSON
    private final Pose startPose = new Pose(55.27, 7.95, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(58.72, 11.89, Math.toRadians(114));
    private final Pose samplePose = new Pose(13.71, 34.19, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(58.72, 11.89, Math.toRadians(114));
    private final Pose pushStartPose = new Pose(7.740, 25.317, Math.toRadians(270));
    private final Pose pushEndPose = new Pose(8.063, 9.837, Math.toRadians(270));
    private final Pose shootPose3 = new Pose(58.72, 11.89, Math.toRadians(123));
    private final Pose parkingPose = new Pose(38.862, 12.094, Math.toRadians(90));

    // Control points for Path 2 (BezierCurve)
    private final Pose ctrl1 = new Pose(57.245, 37.088, 0);
    private final Pose ctrl2 = new Pose(61.76, 27.90, 0);
    private final Pose ctrl3 = new Pose(65.31, 37.09, 0);
    
    // Control point for Path 6 (BezierCurve)
    private final Pose ctrl6 = new Pose(35.153, 32.573, 0);

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
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        // Path 2: Shoot Pose -> Sample (BezierCurve with 3 control points)
        path2_toSample = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, ctrl1, ctrl2, ctrl3, samplePose))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        // Path 3: Sample -> Shoot Pose 2 (BezierLine)
        path3_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(samplePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        // Path 4: Shoot Pose -> Push Start (BezierLine)
        path4_toPushStart = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, pushStartPose))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(270))
                .build();

        // Path 5: Push Start -> Push End (BezierLine)
        path5_toPushEnd = follower.pathBuilder()
                .addPath(new BezierLine(pushStartPose, pushEndPose))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        // Path 6: Push End -> Shoot Pose 3 (BezierCurve with 1 control point)
        path6_toShootPose = follower.pathBuilder()
                .addPath(new BezierCurve(pushEndPose, ctrl6, shootPose3))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(123))
                .build();

        // Path 7: Shoot Pose -> Parking (BezierLine)
        path7_toParking = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, parkingPose))
                .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(90))
                .build();
    }
}

