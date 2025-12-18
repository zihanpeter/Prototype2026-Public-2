package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.localization.Pose;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.commands.autocommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

/**
 * Autonomous OpMode for the Blue Alliance (Basket Side).
 * Follows a 9-path sequence to score Preload + 3 Samples.
 */
@Autonomous(name = "Blue Near Auto", group = "Blue")
public class BlueNear extends AutoCommandBase {

    private PathChain path1_scorePreload;
    private PathChain path2_pickupSample1;
    private PathChain path2_5_gate;
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
        return AutoConstants.BLUE_START_POSE;
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
                new AutoDriveCommand(follower, path1_scorePreload),
                
                // Shoot Preload Logic
                new TransitCommand(transit, shooter), // Shoot

                // =========================================================
                // 1. Path 2: Pickup Sample 1
                // =========================================================
                new InstantCommand(() -> {
                    intake.setFullPower(true); // Run at full power (1.0)
                    intake.startIntake(); 
                    shooter.setShooterState(Shooter.ShooterState.STOP); // Save power? Or keep spinning if fast cycle
                }),
                new AutoDriveCommand(follower, path2_pickupSample1),

                // =========================================================
                // 1.5 Path 2.5: Go to Gate
                // =========================================================
                new AutoDriveCommand(follower, path2_5_gate),
                
                // =========================================================
                // 2. Path 3: Score Sample 1
                // =========================================================
                new InstantCommand(() -> {
                    intake.stopIntake();
                    intake.setFullPower(false); // Reset to normal power
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                new AutoDriveCommand(follower, path3_scoreSample1),
                new TransitCommand(transit, shooter), // Shoot Sample 1

                // =========================================================
                // 3. Path 4 & 5: Pickup Sample 2
                // =========================================================
                new InstantCommand(() -> {
                    intake.startIntake();
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new AutoDriveCommand(follower, path4_pickupSample2_part1),
                new AutoDriveCommand(follower, path5_pickupSample2_part2),

                // =========================================================
                // 4. Path 6: Score Sample 2
                // =========================================================
                new InstantCommand(() -> {
                    intake.stopIntake();
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                new AutoDriveCommand(follower, path6_scoreSample2),
                new TransitCommand(transit, shooter), // Shoot Sample 2

                // =========================================================
                // 5. Path 7 & 8: Pickup Sample 3
                // =========================================================
                new InstantCommand(() -> {
                    intake.startIntake();
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new AutoDriveCommand(follower, path7_pickupSample3_part1),
                new AutoDriveCommand(follower, path8_pickupSample3_part2),

                // =========================================================
                // 6. Path 9: Score Sample 3
                // =========================================================
                new InstantCommand(() -> {
                    intake.stopIntake();
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                new AutoDriveCommand(follower, path9_scoreSample3),
                new TransitCommand(transit, shooter), // Shoot Sample 3
                
                // =========================================================
                // 7. Park (Optional)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new AutoDriveCommand(follower, path10_park),
                new InstantCommand(() -> telemetry.addData("Status", "Auto Completed"))
        );
    }

    private void buildPaths() {
        // Path 1: Start -> Basket
        path1_scorePreload = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_START_POSE),
                        new Point(AutoConstants.BLUE_BASKET_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_START_POSE.getHeading(),
                        AutoConstants.BLUE_BASKET_POSE.getHeading()
                )
                .build();

        // Path 2: Basket -> Sample 1
        path2_pickupSample1 = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_BASKET_POSE),
                        new Point(AutoConstants.BLUE_SAMPLE_1_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_BASKET_POSE.getHeading(),
                        AutoConstants.BLUE_SAMPLE_1_POSE.getHeading()
                )
                .build();

        // Path 2.5: Sample 1 -> Gate
        path2_5_gate = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_SAMPLE_1_POSE),
                        new Point(AutoConstants.BLUE_GATE_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_SAMPLE_1_POSE.getHeading(),
                        AutoConstants.BLUE_GATE_POSE.getHeading()
                )
                .build();

        // Path 3: Gate -> Basket
        path3_scoreSample1 = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_GATE_POSE),
                        new Point(AutoConstants.BLUE_BASKET_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_GATE_POSE.getHeading(),
                        AutoConstants.BLUE_BASKET_POSE.getHeading()
                )
                .build();

        // Path 4: Basket -> Sample 2 Intermediate
        path4_pickupSample2_part1 = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_BASKET_POSE),
                        new Point(AutoConstants.BLUE_SAMPLE_2_INTERMEDIATE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_BASKET_POSE.getHeading(),
                        AutoConstants.BLUE_SAMPLE_2_INTERMEDIATE.getHeading()
                )
                .build();

        // Path 5: Sample 2 Intermediate -> Sample 2 Pickup
        path5_pickupSample2_part2 = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_SAMPLE_2_INTERMEDIATE),
                        new Point(AutoConstants.BLUE_SAMPLE_2_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_SAMPLE_2_INTERMEDIATE.getHeading(),
                        AutoConstants.BLUE_SAMPLE_2_POSE.getHeading()
                )
                .build();

        // Path 6: Sample 2 Pickup -> Basket
        path6_scoreSample2 = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_SAMPLE_2_POSE),
                        new Point(AutoConstants.BLUE_BASKET_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_SAMPLE_2_POSE.getHeading(),
                        AutoConstants.BLUE_BASKET_POSE.getHeading()
                )
                .build();

        // Path 7: Basket -> Sample 3 Intermediate
        path7_pickupSample3_part1 = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_BASKET_POSE),
                        new Point(AutoConstants.BLUE_SAMPLE_3_INTERMEDIATE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_BASKET_POSE.getHeading(),
                        AutoConstants.BLUE_SAMPLE_3_INTERMEDIATE.getHeading()
                )
                .build();

        // Path 8: Sample 3 Intermediate -> Sample 3 Pickup
        path8_pickupSample3_part2 = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_SAMPLE_3_INTERMEDIATE),
                        new Point(AutoConstants.BLUE_SAMPLE_3_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_SAMPLE_3_INTERMEDIATE.getHeading(),
                        AutoConstants.BLUE_SAMPLE_3_POSE.getHeading()
                )
                .build();

        // Path 9: Sample 3 Pickup -> Basket
        path9_scoreSample3 = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_SAMPLE_3_POSE),
                        new Point(AutoConstants.BLUE_BASKET_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_SAMPLE_3_POSE.getHeading(),
                        AutoConstants.BLUE_BASKET_POSE.getHeading()
                )
                .build();
                
        // Path 10: Park (Optional)
        path10_park = follower.pathBuilder()
                .addPath(
                        new Point(AutoConstants.BLUE_BASKET_POSE),
                        new Point(AutoConstants.BLUE_PARK_POSE)
                )
                .setLinearHeadingInterpolation(
                        AutoConstants.BLUE_BASKET_POSE.getHeading(),
                        AutoConstants.BLUE_PARK_POSE.getHeading()
                )
                .build();
    }
}

