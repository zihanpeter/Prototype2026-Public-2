package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;

/**
 * Loop Test Auto
 * Cycle between (72, 72, 90) and (90, 90, 270)
 */
@Autonomous(name = "Loop Test Auto", group = "Test")
public class LoopTestAuto extends LinearOpMode {

    private Follower follower;
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    private final Pose endPose = new Pose(90, 90, Math.toRadians(270));

    private PathChain toEnd, toStart;
    private boolean goingToEnd = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Init Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build Paths
        buildPaths();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();

        waitForStart();

        // Start first path
        follower.followPath(toEnd);
        goingToEnd = true;

        while (opModeIsActive()) {
            follower.update();

            // Check if path is finished
            if (!follower.isBusy()) {
                // Short delay to let robot settle (optional)
                // sleep(500); 

                if (goingToEnd) {
                    // Just finished going to End, now go back to Start
                    follower.followPath(toStart);
                    goingToEnd = false;
                } else {
                    // Just finished going to Start, now go to End
                    follower.followPath(toEnd);
                    goingToEnd = true;
                }
            }

            // Telemetry
            telemetry.addData("Going To", goingToEnd ? "End (90,90,270)" : "Start (72,72,90)");
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Busy", follower.isBusy());
            telemetry.update();
        }
    }

    private void buildPaths() {
        toEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(endPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

        toStart = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(endPose),
                        new Point(startPose)
                ))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();
    }
}

