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
 * Autonomous OpMode for the Blue Alliance Far side (Playoff version).
 * Simplified: removed intake detour, goes directly to push after first shot.
 */
@Autonomous(name = "Blue Far Playoff", group = "Far")
public class BlueFarPlayoff extends AutoCommandBase {

    // Path chains (5 paths)
    private PathChain path1_toShootPose;
    private PathChain path2_toPushStart;
    private PathChain path3_toPushEnd;
    private PathChain path4_toShootPose3;
    private PathChain path5_toPark;

    // Poses
    private final Pose startPose = new Pose(55.27, 7.95, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(61, 11.89, Math.toRadians(112.5));
    private final Pose pushStartPose = new Pose(4, 25.316909294512882, Math.toRadians(270));
    private final Pose pushEndPose = new Pose(5.062709966405375, 9.83650615901455, Math.toRadians(270));
    private final Pose shootPose3 = new Pose(61, 11.89, Math.toRadians(123));
    private final Pose parkPose = new Pose(38.86226203807391, 12.094064949608057, Math.toRadians(90));

    // Control point for curve (Path 4)
    private final Pose ctrl6 = new Pose(35.15341545352744, 32.57334826427771, 0);

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        return new SequentialCommandGroup(
                // Initialize: Start intake
                new InstantCommand(() -> intake.startIntake()),

                // =========================================================
                // Path 1: Start -> Shoot Pose 1 (Preload shot - 3 balls)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.FAST);
                    intake.setFastShooting(true);
                }),
                new AutoDriveCommand(follower, path1_toShootPose, 3000),
                new ParallelDeadlineGroup(
                        new AutoTransitCommand(transit, shooter, follower, shootPose1, intake).withTimeout(3500),
                        new HoldPositionCommand(follower)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),

                // =========================================================
                // Path 2: Shoot Pose 1 -> Push Start (直接去推球)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFastShooting(false);
                }),
                new AutoDriveCommand(follower, path2_toPushStart, 4000),

                // =========================================================
                // Path 3: Push Start -> Push End (intake at fullPower)
                // =========================================================
                new InstantCommand(() -> intake.setFullPower(true)),
                new AutoDriveCommand(follower, path3_toPushEnd, 3000),
                new InstantCommand(() -> intake.setFullPower(false)),

                // =========================================================
                // Path 4: Push End -> Shoot Pose 3 (Final shot)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.FAST);
                    intake.setFastShooting(true);
                }),
                new AutoDriveCommand(follower, path4_toShootPose3, 5000),
                new ParallelDeadlineGroup(
                        new AutoTransitCommand(transit, shooter, follower, shootPose3, intake).withTimeout(2000),
                        new HoldPositionCommand(follower)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Shoot Pose 1 (BezierLine)
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112.5))
                .build();

        // Path 2: Shoot Pose 1 -> Push Start (BezierLine)
        path2_toPushStart = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, pushStartPose))
                .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(270))
                .build();

        // Path 3: Push Start -> Push End (BezierLine)
        path3_toPushEnd = follower.pathBuilder()
                .addPath(new BezierLine(pushStartPose, pushEndPose))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                .build();

        // Path 4: Push End -> Shoot Pose 3 (BezierCurve with 1 control point)
        path4_toShootPose3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pushEndPose,
                        ctrl6,
                        shootPose3))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(123))
                .build();

        // Path 5: Shoot Pose 3 -> Park (BezierLine)
        path5_toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, parkPose))
                .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(90))
                .build();
    }
}
