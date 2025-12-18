package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.geometry.Pose;

/**
 * Stores coordinates and poses for Autonomous OpModes.
 * Generated from Pedro Pathing JSON data for Blue Basket Auto.
 * Red poses are calculated by mirroring Blue poses across x=72.
 */
public class AutoConstants {

    // Helper to convert degrees to radians
    public static double toRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    // =========================================================
    // Start Pose
    // =========================================================
    // Start Point: x: 40.09, y: 136.78, startDeg: 90
    public static final Pose BLUE_START_POSE = new Pose(40.09, 136.78, toRadians(90));
    public static final Pose RED_START_POSE = new Pose(103.91, 136.78, toRadians(90));

    // =========================================================
    // Scoring Pose (Basket)
    // =========================================================
    // Score Point: x: 67.88, y: 75.72, endDeg: 135
    public static final Pose BLUE_BASKET_POSE = new Pose(67.88, 75.72, toRadians(135));
    public static final Pose RED_BASKET_POSE = new Pose(76.12, 75.72, toRadians(45));

    // =========================================================
    // Sample Pickup Poses
    // =========================================================
    
    // Sample 1 (Rightmost/First one): x: 23.65, y: 84.14, endDeg: 180
    public static final Pose BLUE_SAMPLE_1_POSE = new Pose(23.65, 84.14, toRadians(180));
    public static final Pose RED_SAMPLE_1_POSE = new Pose(120.35, 84.14, toRadians(0));

    // Sample 2 (Middle one)
    // Path 4 leads to an intermediate point, Path 5 leads to the pickup
    public static final Pose BLUE_SAMPLE_2_INTERMEDIATE = new Pose(42.49, 59.95, toRadians(180));
    public static final Pose RED_SAMPLE_2_INTERMEDIATE = new Pose(101.51, 59.95, toRadians(0));
    
    public static final Pose BLUE_SAMPLE_2_POSE = new Pose(24.05, 59.82, toRadians(180));
    public static final Pose RED_SAMPLE_2_POSE = new Pose(119.95, 59.82, toRadians(0));

    // Sample 3 (Leftmost/Last one)
    // Path 7 leads to an intermediate point, Path 8 leads to the pickup
    public static final Pose BLUE_SAMPLE_3_INTERMEDIATE = new Pose(42.09, 36.03, toRadians(180));
    public static final Pose RED_SAMPLE_3_INTERMEDIATE = new Pose(101.91, 36.03, toRadians(0));
    
    public static final Pose BLUE_SAMPLE_3_POSE = new Pose(23.92, 35.63, toRadians(180));
    public static final Pose RED_SAMPLE_3_POSE = new Pose(120.08, 35.63, toRadians(0));

    // =========================================================
    // Gate Poses (Added per user request)
    // =========================================================
    public static final Pose BLUE_GATE_POSE = new Pose(15.266, 69.725, toRadians(0));
    public static final Pose RED_GATE_POSE = new Pose(128.734, 69.725, toRadians(180));

    // =========================================================
    // Parking Pose (Optional/TBD)
    // =========================================================
    // Usually park in ascent area or observation zone
    public static final Pose BLUE_PARK_POSE = new Pose(105.477, 33.323, toRadians(90));
    public static final Pose RED_PARK_POSE = new Pose(38.523, 33.323, toRadians(90));
}
