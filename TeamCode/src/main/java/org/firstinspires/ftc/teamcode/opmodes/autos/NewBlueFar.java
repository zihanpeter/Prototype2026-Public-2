package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoAlignCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Autonomous OpMode for the Blue Alliance (Far Side) - New Version.
 * Phase 1: Start -> Shoot -> Sample -> Shoot (once)
 * Phase 2: BlueFar loop (Shoot -> Sample1 -> PushZone -> Sample2 -> Shoot) infinite
 */
@Autonomous(name = "New Blue Far Auto", group = "Blue")
public class NewBlueFar extends AutoCommandBase {

    // Path chains - Phase 1 (new paths)
    private PathChain path1_toShootPose;
    private PathChain path2_toSamplePose;
    private PathChain path3_toShootPose;

    // Path chains - Phase 2 (BlueFar loop paths)
    private PathChain path4_toSample1;
    private PathChain path5_toPushZone;
    private PathChain path6_toSample2;
    private PathChain path7_toFinalShoot;

    // Poses from JSON - Phase 1 (updated)
    private final Pose startPose = new Pose(55.269, 7.953, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(58.716, 11.890, Math.toRadians(114));
    private final Pose samplePose = new Pose(13.707, 34.186, Math.toRadians(180)); // Updated
    private final Pose shootPose2 = new Pose(59.019, 11.449, Math.toRadians(114));

    // Poses from BlueFar - Phase 2
    private final Pose sample1Pose = new Pose(11.303, 10.569, Math.toRadians(180));
    private final Pose pushZone = new Pose(30.679, 10.569, Math.toRadians(180));
    private final Pose sample2Pose = new Pose(11.450, 10.569, Math.toRadians(180));
    private final Pose finalShootPose = new Pose(58.716, 11.743, Math.toRadians(114));

    // Control points for Path 2 (BezierCurve with 3 control points) - updated
    private final Pose controlPoint1 = new Pose(64.824, 51.763, 0);
    private final Pose controlPoint2 = new Pose(61.760, 27.897, 0);
    private final Pose controlPoint3 = new Pose(65.308, 37.088, 0);

    @Override
    public Pose getStartPose() {
        return startPose;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        // Phase 2 Loop: BlueFar cycle (infinite repeat)
        Command blueFarLoop = new RepeatCommand(
                new SequentialCommandGroup(
                        // Path 4: Shoot -> Sample 1
                        new InstantCommand(() -> {
                            shooter.setShooterState(Shooter.ShooterState.STOP);
                            intake.setFastShooting(false);
                        }),
                        new AutoDriveCommand(follower, path4_toSample1),

                        // Path 5: Sample 1 -> Push Zone
                        new AutoDriveCommand(follower, path5_toPushZone),

                        // Path 6: Push Zone -> Sample 2
                        new AutoDriveCommand(follower, path6_toSample2),

                        // Path 7: Sample 2 -> Shoot
                        new InstantCommand(() -> {
                            shooter.setShooterState(Shooter.ShooterState.FAST);
                            intake.setFastShooting(true);
                        }),
                        new AutoDriveCommand(follower, path7_toFinalShoot),
                        new AutoAlignCommand(drive, vision).withTimeout(1000),  // Auto-aim before shooting
                        new TransitCommand(transit, shooter) // Shoots 3 times then finishes
                )
        );

        return new SequentialCommandGroup(
                // =========================================================
                // Initialize: Start intake
                // =========================================================
                new InstantCommand(() -> intake.startIntake()),

                // =========================================================
                // Phase 1: JSON paths (执行一次，不循环)
                // =========================================================
                
                // Path 1: Start -> Shoot Pose (Preload)
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.FAST);
                    intake.setFastShooting(true);
                }),
                new AutoDriveCommand(follower, path1_toShootPose),
                new AutoAlignCommand(drive, vision).withTimeout(1000),  // Auto-aim before shooting
                new TransitCommand(transit, shooter), // 射球 3 次

                // Path 2: Shoot -> Sample (曲线取球)
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                    intake.setFastShooting(false);
                }),
                new AutoDriveCommand(follower, path2_toSamplePose),

                // Path 3: Sample -> Shoot (返回射球)
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.FAST);
                    intake.setFastShooting(true);
                }),
                new AutoDriveCommand(follower, path3_toShootPose),
                new AutoAlignCommand(drive, vision).withTimeout(1000),  // Auto-aim before shooting
                new TransitCommand(transit, shooter), // 射球 3 次

                // =========================================================
                // Phase 2: BlueFar loop (无限循环)
                // =========================================================
                blueFarLoop
        );
    }

    private void buildPaths() {
        // =========================================================
        // Phase 1: New paths
        // =========================================================
        
        // Path 1: Start -> Shoot Pose (BezierLine)
        path1_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        // Path 2: Shoot Pose -> Sample Pose (BezierCurve with 3 control points)
        path2_toSamplePose = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, controlPoint1, controlPoint2, controlPoint3, samplePose))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        // Path 3: Sample Pose -> Shoot Pose (BezierLine)
        path3_toShootPose = follower.pathBuilder()
                .addPath(new BezierLine(samplePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        // =========================================================
        // Phase 2: BlueFar loop paths
        // =========================================================
        
        // Path 4: Shoot Pose -> Sample 1
        path4_toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, sample1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        // Path 5: Sample 1 -> Push Zone
        path5_toPushZone = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, pushZone))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 6: Push Zone -> Sample 2
        path6_toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(pushZone, sample2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 7: Sample 2 -> Shoot Pose
        path7_toFinalShoot = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, finalShootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();
    }
}

