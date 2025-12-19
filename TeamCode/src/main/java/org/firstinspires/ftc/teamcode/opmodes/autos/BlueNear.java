package org.firstinspires.ftc.teamcode.opmodes.autos;

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
 * Autonomous OpMode for the Blue Alliance (Basket Side).
 * Hardcoded paths updated from user JSON.
 */
@Autonomous(name = "Blue Near Auto", group = "Blue")
public class BlueNear extends AutoCommandBase {

    private PathChain path1_scorePreload; // (Unused now)
    private PathChain path2_pickupSample1_part1;
    private PathChain path2_pickupSample1_part2;
    private PathChain path3_scoreSample1;
    private PathChain path4_pickupSample2_part1;
    private PathChain path5_pickupSample2_part2;
    private PathChain path6_scoreSample2;
    private PathChain path7_pickupSample3_part1;
    private PathChain path8_pickupSample3_part2;
    private PathChain path9_scoreSample3;
    private PathChain path10_park;

    @Override
    public Pose getStartPose() {
        return new Pose(24.854, 128.901, Math.toRadians(144));
    }

    @Override
    public Command runAutoCommand() {
        buildPaths();

        return new SequentialCommandGroup(
                // =========================================================
                // 0. Setup & Preload Shoot (Stationary)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                // Shoot Preload Logic
                new TransitCommand(transit, shooter).withTimeout(800), // Shoot immediately at Start Pose

                // =========================================================
                // 0b. Start Intake
                // =========================================================
                new InstantCommand(() -> {
                    intake.startIntake(); // Start continuous intake at 0.5 power
                }),

                // =========================================================
                // 1. Path 2: Pickup Sample 1 (Start -> Intermediate -> Pickup)
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP); 
                }),
                new AutoDriveCommand(follower, path2_pickupSample1_part1).withTimeout(3000),
                new AutoDriveCommand(follower, path2_pickupSample1_part2).withTimeout(3000),
                
                // =========================================================
                // 2. Path 3: Score Sample 1
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                new AutoDriveCommand(follower, path3_scoreSample1).withTimeout(3000),
                new TransitCommand(transit, shooter).withTimeout(800), // Shoot Sample 1

                // =========================================================
                // 3. Path 4 & 5: Pickup Sample 2
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new AutoDriveCommand(follower, path4_pickupSample2_part1).withTimeout(3000),
                new AutoDriveCommand(follower, path5_pickupSample2_part2).withTimeout(3000),

                // =========================================================
                // 4. Path 6: Score Sample 2
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.MID);
                }),
                new AutoDriveCommand(follower, path6_scoreSample2).withTimeout(3000),
                new TransitCommand(transit, shooter).withTimeout(800), // Shoot Sample 2

                // =========================================================
                // 5. Path 7 & 8: Pickup Sample 3
                // =========================================================
                new InstantCommand(() -> {
                    shooter.setShooterState(Shooter.ShooterState.STOP);
                }),
                new AutoDriveCommand(follower, path7_pickupSample3_part1).withTimeout(3000),
                new AutoDriveCommand(follower, path8_pickupSample3_part2).withTimeout(3000),

                // =========================================================
                // 6. Path 9: Score Sample 3
                // =========================================================
                new InstantCommand(() -> {
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
                // new AutoDriveCommand(follower, path10_park).withTimeout(3000),
                new InstantCommand(() -> telemetry.addData("Status", "Auto Completed"))
        );
    }

    private void buildPaths() {
        // Define Poses locally from User JSON
        // Start: x:24.854, y:128.901, deg:144
        Pose startPose = new Pose(24.854, 128.901, Math.toRadians(144));
        
        // Basket: x:67.88, y:75.75, deg:135 (Used in Paths 3, 6, 9)
        Pose basketPose = new Pose(67.88, 75.75, Math.toRadians(135));
        
        // Sample 1 Intermediate: x:43.055, y:83.852, deg:135 (End of Path 1)
        Pose sample1Intermediate = new Pose(43.055, 83.852, Math.toRadians(135));
        
        // Sample 1 Pickup: x:23.785, y:83.870, deg:180 (End of Path 2)
        Pose sample1Pose = new Pose(23.785, 83.870, Math.toRadians(180));
        
        // Sample 2 Intermediate: x:42.492, y:59.952, deg:180 (End of Path 4)
        Pose sample2Intermediate = new Pose(42.492, 59.952, Math.toRadians(180));
        
        // Sample 2 Pickup: x:24.052, y:59.819, deg:180 (End of Path 5)
        Pose sample2Pose = new Pose(24.052, 59.819, Math.toRadians(180));
        
        // Sample 3 Intermediate: x:42.091, y:36.034, deg:180 (End of Path 7)
        Pose sample3Intermediate = new Pose(42.091, 36.034, Math.toRadians(180));
        
        // Sample 3 Pickup: x:23.918, y:35.633, deg:180 (End of Path 8)
        Pose sample3Pose = new Pose(23.918, 35.633, Math.toRadians(180));
        
        // Park: Relative to basket? Original Park was (60, 96). 
        // Keeping original park or maybe adjust based on basket.
        Pose parkPose = new Pose(60, 96, Math.toRadians(90));

        // Path 2 Part 1: Start -> Sample 1 Intermediate
        // JSON Path 1: Start 144 -> End 135
        path2_pickupSample1_part1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, sample1Intermediate))
                .setLinearHeadingInterpolation(startPose.getHeading(), sample1Intermediate.getHeading())
                .build();

        // Path 2 Part 2: Sample 1 Intermediate -> Sample 1 Pickup
        // JSON Path 2: Start 135 -> End 180
        path2_pickupSample1_part2 = follower.pathBuilder()
                .addPath(new BezierLine(sample1Intermediate, sample1Pose))
                .setLinearHeadingInterpolation(sample1Intermediate.getHeading(), sample1Pose.getHeading())
                .build();

        // Path 3: Sample 1 -> Basket
        // JSON Path 3: Start 180 -> End 135
        path3_scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(sample1Pose, basketPose))
                .setLinearHeadingInterpolation(sample1Pose.getHeading(), basketPose.getHeading())
                .build();

        // Path 4: Basket -> Sample 2 Intermediate
        // JSON Path 4: Start 135 -> End 180
        path4_pickupSample2_part1 = follower.pathBuilder()
                .addPath(new BezierLine(basketPose, sample2Intermediate))
                .setLinearHeadingInterpolation(basketPose.getHeading(), sample2Intermediate.getHeading())
                .build();

        // Path 5: Sample 2 Intermediate -> Sample 2 Pickup
        // JSON Path 5: Start 180 -> End 180
        path5_pickupSample2_part2 = follower.pathBuilder()
                .addPath(new BezierLine(sample2Intermediate, sample2Pose))
                .setLinearHeadingInterpolation(sample2Intermediate.getHeading(), sample2Pose.getHeading())
                .build();

        // Path 6: Sample 2 Pickup -> Basket
        // JSON Path 6: Start 180 -> End 135
        path6_scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(sample2Pose, basketPose))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), basketPose.getHeading())
                .build();

        // Path 7: Basket -> Sample 3 Intermediate
        // JSON Path 7: Start 135 -> End 180
        path7_pickupSample3_part1 = follower.pathBuilder()
                .addPath(new BezierLine(basketPose, sample3Intermediate))
                .setLinearHeadingInterpolation(basketPose.getHeading(), sample3Intermediate.getHeading())
                .build();

        // Path 8: Sample 3 Intermediate -> Sample 3 Pickup
        // JSON Path 8: Start 180 -> End 180
        path8_pickupSample3_part2 = follower.pathBuilder()
                .addPath(new BezierLine(sample3Intermediate, sample3Pose))
                .setLinearHeadingInterpolation(sample3Intermediate.getHeading(), sample3Pose.getHeading())
                .build();

        // Path 9: Sample 3 Pickup -> Basket
        // JSON Path 9: Start 180 -> End 135
        path9_scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(sample3Pose, basketPose))
                .setLinearHeadingInterpolation(sample3Pose.getHeading(), basketPose.getHeading())
                .build();
                
        // Path 10: Park (Optional)
        path10_park = follower.pathBuilder()
                .addPath(new BezierLine(basketPose, parkPose))
                .setLinearHeadingInterpolation(basketPose.getHeading(), parkPose.getHeading())
                .build();
    }
}
