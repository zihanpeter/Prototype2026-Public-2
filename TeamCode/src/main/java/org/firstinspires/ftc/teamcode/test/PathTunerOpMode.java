package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;

/**
 * Path Tuner OpMode
 * Allows dynamic path testing via FTC Dashboard.
 * Supports both BezierLine and BezierCurve.
 *
 * Instructions:
 * 1. Open FTC Dashboard (192.168.43.1:8080/dash).
 * 2. Edit start/end/control point values in the Configuration sidebar.
 * 3. Set pathType: 0 = BezierLine, 1 = BezierCurve (1 ctrl), 2 = BezierCurve (2 ctrl), 3 = BezierCurve (3 ctrl)
 * 4. Init this OpMode. The robot will set its pose to 'startPose'.
 * 5. Press Start. The robot will drive to 'endPose'.
 * 6. Press A to re-run the path.
 */
@Config
@TeleOp(name = "Path Tuner", group = "Test")
public class PathTunerOpMode extends OpMode {

    private Follower follower;
    private FtcDashboard dashboard;

    // Path Type: 0 = Line, 1 = Curve (1 ctrl), 2 = Curve (2 ctrl), 3 = Curve (3 ctrl)
    public static int pathType = 0;

    // Start Pose
    public static double startX = 25.509;
    public static double startY = 129.474;
    public static double startHeadingDeg = 144.0;

    // End Pose
    public static double endX = 37.200;
    public static double endY = 110.342;
    public static double endHeadingDeg = 135.0;

    // Control Point 1 (for BezierCurve)
    public static double ctrl1X = 50.0;
    public static double ctrl1Y = 120.0;

    // Control Point 2 (for BezierCurve with 2+ control points)
    public static double ctrl2X = 60.0;
    public static double ctrl2Y = 100.0;

    // Control Point 3 (for BezierCurve with 3 control points)
    public static double ctrl3X = 45.0;
    public static double ctrl3Y = 90.0;

    private PathChain testPath;
    private boolean pathRunning = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        follower = Constants.createFollower(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Path Type", getPathTypeName());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Visualize the intended path on Dashboard before starting
        TelemetryPacket packet = new TelemetryPacket();

        // Draw path based on type
        drawExpectedPath(packet);

        // Add Data to Dashboard
        packet.put("Path Type", getPathTypeName());
        packet.put("Start", String.format("(%.1f, %.1f, %.0f°)", startX, startY, startHeadingDeg));
        packet.put("End", String.format("(%.1f, %.1f, %.0f°)", endX, endY, endHeadingDeg));

        dashboard.sendTelemetryPacket(packet);

        // Update Driver Station Telemetry
        telemetry.addData("Path Type", getPathTypeName());
        telemetry.addData("Start", String.format("(%.1f, %.1f, %.0f°)", startX, startY, startHeadingDeg));
        telemetry.addData("End", String.format("(%.1f, %.1f, %.0f°)", endX, endY, endHeadingDeg));
        if (pathType >= 1) telemetry.addData("Ctrl1", String.format("(%.1f, %.1f)", ctrl1X, ctrl1Y));
        if (pathType >= 2) telemetry.addData("Ctrl2", String.format("(%.1f, %.1f)", ctrl2X, ctrl2Y));
        if (pathType >= 3) telemetry.addData("Ctrl3", String.format("(%.1f, %.1f)", ctrl3X, ctrl3Y));
        telemetry.update();
    }

    @Override
    public void start() {
        buildAndFollowPath();
    }

    @Override
    public void loop() {
        follower.update();

        // Create Packet
        TelemetryPacket packet = new TelemetryPacket();

        // Draw the expected path
        drawExpectedPath(packet);

        // Draw Robot manually
        Pose currentPose = follower.getPose();
        packet.fieldOverlay()
                .setFill("blue")
                .fillCircle(currentPose.getX(), currentPose.getY(), 2)
                .setStroke("black")
                .strokeLine(currentPose.getX(), currentPose.getY(),
                        currentPose.getX() + 7 * Math.cos(currentPose.getHeading()),
                        currentPose.getY() + 7 * Math.sin(currentPose.getHeading()));

        // Dashboard Telemetry
        packet.put("Path Running", follower.isBusy());
        packet.put("X", currentPose.getX());
        packet.put("Y", currentPose.getY());
        packet.put("Heading", Math.toDegrees(currentPose.getHeading()));

        double distToEnd = Math.hypot(endX - currentPose.getX(), endY - currentPose.getY());
        packet.put("Dist to End", distToEnd);

        dashboard.sendTelemetryPacket(packet);

        // Driver Station Telemetry
        telemetry.addData("Path Running", follower.isBusy());
        telemetry.addData("Current X", currentPose.getX());
        telemetry.addData("Current Y", currentPose.getY());
        telemetry.addData("Current Deg", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Dist to End", distToEnd);
        telemetry.addData("Press A", "to re-run path");
        telemetry.update();

        // Press A to re-run path
        if (gamepad1.a && !pathRunning) {
            buildAndFollowPath();
        }

        if (!follower.isBusy()) {
            pathRunning = false;
        }
    }

    private void buildAndFollowPath() {
        Pose startPose = new Pose(startX, startY, Math.toRadians(startHeadingDeg));
        Pose endPose = new Pose(endX, endY, Math.toRadians(endHeadingDeg));

        follower.setStartingPose(startPose);

        switch (pathType) {
            case 0: // BezierLine
                testPath = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, endPose))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                        .build();
                break;

            case 1: // BezierCurve with 1 control point
                testPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                startPose,
                                new Pose(ctrl1X, ctrl1Y, 0),
                                endPose
                        ))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                        .build();
                break;

            case 2: // BezierCurve with 2 control points
                testPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                startPose,
                                new Pose(ctrl1X, ctrl1Y, 0),
                                new Pose(ctrl2X, ctrl2Y, 0),
                                endPose
                        ))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                        .build();
                break;

            case 3: // BezierCurve with 3 control points
            default:
                testPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                startPose,
                                new Pose(ctrl1X, ctrl1Y, 0),
                                new Pose(ctrl2X, ctrl2Y, 0),
                                new Pose(ctrl3X, ctrl3Y, 0),
                                endPose
                        ))
                        .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                        .build();
                break;
        }

        follower.followPath(testPath);
        pathRunning = true;
    }

    private void drawExpectedPath(TelemetryPacket packet) {
        // Debug: Draw a test cross at center to verify Dashboard is working
        packet.fieldOverlay()
                .setStroke("yellow")
                .strokeLine(60, 60, 84, 84)
                .strokeLine(60, 84, 84, 60);
        
        // Draw start point (blue) and end point (red)
        packet.fieldOverlay()
                .setStroke("blue")
                .strokeCircle(startX, startY, 2)
                .setStroke("red")
                .strokeCircle(endX, endY, 2);

        // Draw path based on type
        if (pathType == 0) {
            // Simple line
            packet.fieldOverlay()
                    .setStroke("green")
                    .strokeLine(startX, startY, endX, endY);
        } else {
            // Draw Bezier curve approximation (sample points along curve)
            drawBezierCurve(packet);

            // Draw control points (orange)
            packet.fieldOverlay().setStroke("orange");
            if (pathType >= 1) {
                packet.fieldOverlay().strokeCircle(ctrl1X, ctrl1Y, 1.5);
                packet.fieldOverlay().setStroke("gray").strokeLine(startX, startY, ctrl1X, ctrl1Y);
            }
            if (pathType >= 2) {
                packet.fieldOverlay().setStroke("orange").strokeCircle(ctrl2X, ctrl2Y, 1.5);
            }
            if (pathType >= 3) {
                packet.fieldOverlay().setStroke("orange").strokeCircle(ctrl3X, ctrl3Y, 1.5);
                packet.fieldOverlay().setStroke("gray").strokeLine(ctrl3X, ctrl3Y, endX, endY);
            }
        }
    }

    private void drawBezierCurve(TelemetryPacket packet) {
        // Sample 20 points along the Bezier curve and draw line segments
        int samples = 20;
        double[] lastPoint = {startX, startY};

        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            double[] point = calculateBezierPoint(t);

            packet.fieldOverlay()
                    .setStroke("green")
                    .strokeLine(lastPoint[0], lastPoint[1], point[0], point[1]);

            lastPoint = point;
        }
    }

    private double[] calculateBezierPoint(double t) {
        double x, y;

        switch (pathType) {
            case 1: // Quadratic Bezier (1 control point)
                x = Math.pow(1 - t, 2) * startX + 2 * (1 - t) * t * ctrl1X + Math.pow(t, 2) * endX;
                y = Math.pow(1 - t, 2) * startY + 2 * (1 - t) * t * ctrl1Y + Math.pow(t, 2) * endY;
                break;

            case 2: // Cubic Bezier (2 control points)
                x = Math.pow(1 - t, 3) * startX + 3 * Math.pow(1 - t, 2) * t * ctrl1X
                        + 3 * (1 - t) * Math.pow(t, 2) * ctrl2X + Math.pow(t, 3) * endX;
                y = Math.pow(1 - t, 3) * startY + 3 * Math.pow(1 - t, 2) * t * ctrl1Y
                        + 3 * (1 - t) * Math.pow(t, 2) * ctrl2Y + Math.pow(t, 3) * endY;
                break;

            case 3: // Quartic Bezier (3 control points)
            default:
                double u = 1 - t;
                x = Math.pow(u, 4) * startX + 4 * Math.pow(u, 3) * t * ctrl1X
                        + 6 * Math.pow(u, 2) * Math.pow(t, 2) * ctrl2X
                        + 4 * u * Math.pow(t, 3) * ctrl3X + Math.pow(t, 4) * endX;
                y = Math.pow(u, 4) * startY + 4 * Math.pow(u, 3) * t * ctrl1Y
                        + 6 * Math.pow(u, 2) * Math.pow(t, 2) * ctrl2Y
                        + 4 * u * Math.pow(t, 3) * ctrl3Y + Math.pow(t, 4) * endY;
                break;
        }

        return new double[]{x, y};
    }

    private String getPathTypeName() {
        switch (pathType) {
            case 0: return "BezierLine";
            case 1: return "BezierCurve (1 ctrl)";
            case 2: return "BezierCurve (2 ctrl)";
            case 3: return "BezierCurve (3 ctrl)";
            default: return "Unknown";
        }
    }
}
