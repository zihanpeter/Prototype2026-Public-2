package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;

/**
 * Path Tuner OpMode
 * Allows dynamic path testing via FTC Dashboard.
 * 
 * Instructions:
 * 1. Open FTC Dashboard (192.168.43.1:8080/dash).
 * 2. Edit 'startPose' and 'endPose' values in the Configuration sidebar.
 * 3. Init this OpMode. The robot will set its pose to 'startPose'.
 * 4. Press Start. The robot will drive to 'endPose'.
 * 5. To test again, stop and restart the OpMode (or press A to reset if implemented).
 */
@Config
@TeleOp(name = "Path Tuner", group = "Test")
public class PathTunerOpMode extends OpMode {

    private Follower follower;
    private FtcDashboard dashboard;

    // Dashboard Configurable Variables
    public static double startX = 24.854;
    public static double startY = 128.901;
    public static double startHeadingDeg = 144.0;

    public static double endX = 43.055;
    public static double endY = 83.852;
    public static double endHeadingDeg = 135.0;

    private PathChain testPath;
    private boolean pathRunning = false;

    @Override
    public void init() {
        // Setup Dashboard
        dashboard = FtcDashboard.getInstance();
        // NOTE: We do NOT use MultipleTelemetry here to avoid packet conflicts when manually drawing.
        // Standard telemetry is used for Driver Station.

        // Initialize Follower
        follower = Constants.createFollower(hardwareMap);

        // Initial Pose Setup
        Pose startPose = new Pose(startX, startY, Math.toRadians(startHeadingDeg));
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", startPose);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Visualize the intended path on Dashboard before starting
        Pose currentStart = new Pose(startX, startY, Math.toRadians(startHeadingDeg));
        Pose currentEnd = new Pose(endX, endY, Math.toRadians(endHeadingDeg));
        
        // Create Packet for Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        
        // Draw expected path (Green Line)
        packet.fieldOverlay()
                .setStroke("green")
                .strokeLine(currentStart.getX(), currentStart.getY(), currentEnd.getX(), currentEnd.getY())
                .setStroke("blue")
                .strokeCircle(currentStart.getX(), currentStart.getY(), 2) // Start
                .setStroke("red")
                .strokeCircle(currentEnd.getX(), currentEnd.getY(), 2);   // End
        
        // Add Data to Dashboard
        packet.put("Config Start", currentStart.toString());
        packet.put("Config End", currentEnd.toString());
        
        // Send Packet
        dashboard.sendTelemetryPacket(packet);
        
        // Update Driver Station Telemetry
        telemetry.addData("Config Start", currentStart);
        telemetry.addData("Config End", currentEnd);
        telemetry.update();
    }

    @Override
    public void start() {
        // Build the path based on final config values
        Pose startPose = new Pose(startX, startY, Math.toRadians(startHeadingDeg));
        Pose endPose = new Pose(endX, endY, Math.toRadians(endHeadingDeg));

        // Ensure follower knows where it starts
        follower.setStartingPose(startPose);

        // Build Path
        testPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

        // Start Following
        follower.followPath(testPath);
        pathRunning = true;
    }

    @Override
    public void loop() {
        follower.update();

        // Create Packet
        TelemetryPacket packet = new TelemetryPacket();

        // Draw the path again (it might disappear after init)
        packet.fieldOverlay()
                .setStroke("green")
                .strokeLine(startX, startY, endX, endY);
        
        // Draw Robot (Optional, Follower might draw it itself, but we draw extra to be sure)
        follower.drawOnDashboard(packet, null); // Draw follower debug info

        // Dashboard Telemetry
        packet.put("Path Running", follower.isBusy());
        packet.put("X", follower.getPose().getX());
        packet.put("Y", follower.getPose().getY());
        packet.put("Heading", Math.toDegrees(follower.getPose().getHeading()));
        
        // Error calculation
        double distToEnd = Math.hypot(endX - follower.getPose().getX(), endY - follower.getPose().getY());
        packet.put("Dist to End", distToEnd);
        
        // Send to Dashboard
        dashboard.sendTelemetryPacket(packet);

        // Driver Station Telemetry
        telemetry.addData("Path Running", follower.isBusy());
        telemetry.addData("Current X", follower.getPose().getX());
        telemetry.addData("Current Y", follower.getPose().getY());
        telemetry.addData("Current Deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Dist to End", distToEnd);
        telemetry.update();
        
        // Optional: Press A to reset and run again
        if (gamepad1.a && !pathRunning) {
             start(); // Re-trigger path
        }
        
        if (!follower.isBusy()) {
            pathRunning = false;
        }
    }
}
