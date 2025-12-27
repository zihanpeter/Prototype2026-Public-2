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
 * Autonomous OpMode for the Blue Alliance Far side (Shanghai version).
 * 7 paths with curves and specific shooting positions.
 */
@Autonomous(name = "Blue Far Shanghai", group = "Far")
public class BlueFarShanghai extends AutoCommandBase {

    // Path chains
    private PathChain path1_toShootPose;
    private PathChain path2_toIntakePose;
    private PathChain path3_toShootPose2;
    private PathChain path4_toPushStart;
    private PathChain path5_toPushEnd;
    private PathChain path6_toShootPose3;
    private PathChain path7_toPark;

    // Poses from JSON
    private final Pose startPose = new Pose(55.27, 7.95, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(58.72, 11.89, Math.toRadians(114));
    private final Pose intakePose = new Pose(13.71, 34.19, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(58.72, 11.89, Math.toRadians(114));
    private final Pose pushStartPose = new Pose(4, 25.316909294512882, Math.toRadians(270));
    private final Pose pushEndPose = new Pose(5.062709966405375, 9.83650615901455, Math.toRadians(270));
    private final Pose shootPose3 = new Pose(58.72, 11.89, Math.toRadians(123)); // Final shooting angle: 123 degrees
    private final Pose parkPose = new Pose(38.86226203807391, 12.094064949608057, Math.toRadians(90));

    // Control points for curves (using Pose with heading=0)
    private final Pose ctrl1 = new Pose(57.245240761478165, 37.088465845464725, 0);
    private final Pose ctrl2 = new Pose(61.76, 27.9, 0);
    private final Pose ctrl3 = new Pose(65.31, 37.09, 0);
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
                // Path 2: Shoot Pose 1 -> Intake Pose (with intake at fullPower)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFastShooting(false);
                    intake.setFullPower(true);
                }),
                new AutoDriveCommand(follower, path2_toIntakePose, 5000),
                new InstantCommand(() -> intake.setFullPower(false)),

                // =========================================================
                // Path 3: Intake Pose -> Shoot Pose 2
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.FAST);
                    intake.setFastShooting(true);
                }),
                new AutoDriveCommand(follower, path3_toShootPose2, 4000),
                new ParallelDeadlineGroup(
                        new AutoTransitCommand(transit, shooter, follower, shootPose2, intake).withTimeout(1300),
                        new HoldPositionCommand(follower)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),

                // =========================================================
                // Path 4: Shoot Pose 2 -> Push Start
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFastShooting(false);
                }),
                new AutoDriveCommand(follower, path4_toPushStart, 4000),

                // =========================================================
                // Path 5: Push Start -> Push End (intake at fullPower)
                // =========================================================
                new InstantCommand(() -> intake.setFullPower(true)),
                new AutoDriveCommand(follower, path5_toPushEnd, 3000),
                new InstantCommand(() -> intake.setFullPower(false)),

                // =========================================================
                // Path 6: Push End -> Shoot Pose 3 (Final shot)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.FAST);
                    intake.setFastShooting(true);
                }),
                new AutoDriveCommand(follower, path6_toShootPose3, 5000),
                new ParallelDeadlineGroup(
                        new AutoTransitCommand(transit, shooter, follower, shootPose3, intake).withTimeout(2000),
                        new HoldPositionCommand(follower)
                ),
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP)),

                // =========================================================
                // Path 7: Shoot Pose 3 -> Park
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFastShooting(false);
                }),
                new AutoDriveCommand(follower, path7_toPark, 3000)
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Shoot Pose 1 (BezierLine)
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        // Path 2: Shoot Pose 1 -> Intake Pose (BezierCurve with 3 control points)
        path2_toIntakePose = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootPose1,
                        ctrl1,
                        ctrl2,
                        ctrl3,
                        intakePose))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        // Path 3: Intake Pose -> Shoot Pose 2 (BezierLine)
        path3_toShootPose2 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        // Path 4: Shoot Pose 2 -> Push Start (BezierLine)
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
        path6_toShootPose3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        pushEndPose,
                        ctrl6,
                        shootPose3))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(123))
                .build();

        // Path 7: Shoot Pose 3 -> Park (BezierLine)
        path7_toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, parkPose))
                .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(90))
                .build();
    }
}

