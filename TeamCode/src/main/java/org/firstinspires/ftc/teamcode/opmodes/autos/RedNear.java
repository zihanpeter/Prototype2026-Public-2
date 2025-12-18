package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_BASKET_POSE;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_GATE_POSE;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_PARK_POSE;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_SAMPLE_1_POSE;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_SAMPLE_2_INTERMEDIATE;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_SAMPLE_2_POSE;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_SAMPLE_3_INTERMEDIATE;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_SAMPLE_3_POSE;
import static org.firstinspires.ftc.teamcode.opmodes.autos.AutoConstants.RED_START_POSE;

import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.paths.PathChain;

import com.pedropathing.geometry.Pose;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Autonomous OpMode for the Red Alliance (Basket Side).
 * Mirrored version of BlueNear.
 */
@Autonomous(name = "Red Near Auto", group = "Red")
public class RedNear extends AutoCommandBase {

    private PathChain path1_scorePreload;
    private PathChain path2_pickupSample1;
    private PathChain path3_scoreSample1;
    private PathChain path4_pickupSample2_part1;
    private PathChain path5_pickupSample2_part2;
    private PathChain path6_scoreSample2;
    private PathChain path7_pickupSample3_part1;
    private PathChain path8_pickupSample3_part2;
    private PathChain path9_scoreSample3;
    // Optional parking path
    private PathChain path10_park;

    @Override
    public com.pedropathing.geometry.Pose getStartPose() {
        return AutoConstants.RED_START_POSE;
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        return new SequentialCommandGroup(
                // =========================================================
                // 0. Setup & Path 1: Move to Score Preload
                // =========================================================
                new InstantCommand(() -> {
                    // Start shooter for preload (MID)
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                // =========================================================
                // 0. Preload & Start Intake
                // =========================================================
                new InstantCommand(() -> {
                    intake.startIntake(); // Start continuous intake
                }),
                new AutoDriveCommand(follower, path1_scorePreload).withTimeout(3000),
                
                // Shoot Preload Logic
                new TransitCommand(transit, shooter).withTimeout(800), // Shoot

                // =========================================================
                // 1. Path 2: Pickup Sample 1
                // =========================================================
                new InstantCommand(() -> {
                    // Intake is already running
                    shooter.setShooterState(Shooter.ShooterState.STOP); 
                }),
                new AutoDriveCommand(follower, path2_pickupSample1).withTimeout(3000),
                
                // =========================================================
                // 2. Path 3: Score Sample 1
                // =========================================================
                new InstantCommand(() -> {
                    // Keep Intake running
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                new AutoDriveCommand(follower, path3_scoreSample1).withTimeout(3000),
                new TransitCommand(transit, shooter).withTimeout(800), // Shoot Sample 1

                // =========================================================
                // 3. Path 4 & 5: Pickup Sample 2
                // =========================================================
                new InstantCommand(() -> {
                    // Intake is already running
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new AutoDriveCommand(follower, path4_pickupSample2_part1).withTimeout(3000),
                new AutoDriveCommand(follower, path5_pickupSample2_part2).withTimeout(3000),

                // =========================================================
                // 4. Path 6: Score Sample 2
                // =========================================================
                new InstantCommand(() -> {
                    // Keep Intake running
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                new AutoDriveCommand(follower, path6_scoreSample2).withTimeout(3000),
                new TransitCommand(transit, shooter).withTimeout(800), // Shoot Sample 2

                // =========================================================
                // 5. Path 7 & 8: Pickup Sample 3
                // =========================================================
                new InstantCommand(() -> {
                    // Intake is already running
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new AutoDriveCommand(follower, path7_pickupSample3_part1).withTimeout(3000),
                new AutoDriveCommand(follower, path8_pickupSample3_part2).withTimeout(3000),

                // =========================================================
                // 6. Path 9: Score Sample 3
                // =========================================================
                new InstantCommand(() -> {
                    // Keep Intake running
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                new AutoDriveCommand(follower, path9_scoreSample3).withTimeout(3000),
                new TransitCommand(transit, shooter).withTimeout(800), // Shoot Sample 3
                
                // =========================================================
                // 7. Park (Optional)
                // =========================================================
                new InstantCommand(() -> {
                    intake.stopIntake();
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new AutoDriveCommand(follower, path10_park).withTimeout(3000),
                new InstantCommand(() -> telemetry.addData("Status", "Auto Completed"))
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Basket
        path1_scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_START_POSE,
                        RED_BASKET_POSE
                ))
                .setLinearHeadingInterpolation(
                        RED_START_POSE.getHeading(),
                        RED_BASKET_POSE.getHeading()
                )
                .build();

        // Path 2: Basket -> Sample 1
        path2_pickupSample1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_BASKET_POSE,
                        RED_SAMPLE_1_POSE
                ))
                .setLinearHeadingInterpolation(
                        RED_BASKET_POSE.getHeading(),
                        RED_SAMPLE_1_POSE.getHeading()
                )
                .build();

        // Path 3: Sample 1 -> Basket
        path3_scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_SAMPLE_1_POSE,
                        RED_BASKET_POSE
                ))
                .setLinearHeadingInterpolation(
                        RED_SAMPLE_1_POSE.getHeading(),
                        RED_BASKET_POSE.getHeading()
                )
                .build();

        // Path 4: Basket -> Sample 2 Intermediate
        path4_pickupSample2_part1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_BASKET_POSE,
                        RED_SAMPLE_2_INTERMEDIATE
                ))
                .setLinearHeadingInterpolation(
                        RED_BASKET_POSE.getHeading(),
                        RED_SAMPLE_2_INTERMEDIATE.getHeading()
                )
                .build();

        // Path 5: Sample 2 Intermediate -> Sample 2 Pickup
        path5_pickupSample2_part2 = follower.pathBuilder()
                .addPath(new BezierLine(
                       RED_SAMPLE_2_INTERMEDIATE,
                        RED_SAMPLE_2_POSE
                ))
                .setLinearHeadingInterpolation(
                        RED_SAMPLE_2_INTERMEDIATE.getHeading(),
                        RED_SAMPLE_2_POSE.getHeading()
                )
                .build();

        // Path 6: Sample 2 Pickup -> Basket
        path6_scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_SAMPLE_2_POSE,
                        RED_BASKET_POSE
                ))
                .setLinearHeadingInterpolation(
                        RED_SAMPLE_2_POSE.getHeading(),
                        RED_BASKET_POSE.getHeading()
                )
                .build();

        // Path 7: Basket -> Sample 3 Intermediate
        path7_pickupSample3_part1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_BASKET_POSE,
                        RED_SAMPLE_3_INTERMEDIATE
                ))
                .setLinearHeadingInterpolation(
                        RED_BASKET_POSE.getHeading(),
                        RED_SAMPLE_3_INTERMEDIATE.getHeading()
                )
                .build();

        // Path 8: Sample 3 Intermediate -> Sample 3 Pickup
        path8_pickupSample3_part2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_SAMPLE_3_INTERMEDIATE,
                        RED_SAMPLE_3_POSE
                ))
                .setLinearHeadingInterpolation(
                        RED_SAMPLE_3_INTERMEDIATE.getHeading(),
                        RED_SAMPLE_3_POSE.getHeading()
                )
                .build();

        // Path 9: Sample 3 Pickup -> Basket
        path9_scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_SAMPLE_3_POSE,
                        RED_BASKET_POSE
                ))
                .setLinearHeadingInterpolation(
                        RED_SAMPLE_3_POSE.getHeading(),
                        RED_BASKET_POSE.getHeading()
                )
                .build();
                
        // Path 10: Park (Optional)
        path10_park = follower.pathBuilder()
                .addPath(new BezierLine(
                        RED_BASKET_POSE,
                       RED_PARK_POSE
                ))
                .setLinearHeadingInterpolation(
                        RED_BASKET_POSE.getHeading(),
                        RED_PARK_POSE.getHeading()
                )
                .build();
    }
}

