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
    // Start Poses
    // =========================================================
    
    // NEAR Side (Basket Side)
    // Start Point: x: 22.716, y: 125.961, startDeg: 144
    public static final Pose BLUE_START_POSE = new Pose(22.716, 125.961, toRadians(144));
    public static final Pose RED_START_POSE = new Pose(121.284, 125.961, toRadians(36));

    // FAR Side (Observation Zone Side)
    // Start Point: x: 47.892, y: 8.063, startDeg: 90
    public static final Pose BLUE_FAR_START_POSE = new Pose(47.892, 8.063, toRadians(90));
    public static final Pose RED_FAR_START_POSE = new Pose(96.108, 8.063, toRadians(90));

    // =========================================================
    // NEAR Side: Scoring & Samples
    // =========================================================
    
    // Scoring Pose (Basket)
    // Score Point: x: 67.88, y: 75.72, endDeg: 135
    public static final Pose BLUE_BASKET_POSE = new Pose(67.88, 75.72, toRadians(135));
    public static final Pose RED_BASKET_POSE = new Pose(76.12, 75.72, toRadians(45));

    // =========================================================
    // FAR Side: Shooting Pose
    // =========================================================
    // Calculated Heading: Angle to (11.127, 136.744) + 90 deg
    // Vector Angle ~113.25 deg, Target Heading ~203.25 deg
    public static final Pose BLUE_FAR_SHOOT_POSE = new Pose(62.889, 16.287, toRadians(203.25));
    public static final Pose RED_FAR_SHOOT_POSE = new Pose(81.111, 16.287, toRadians(-23.25));

    // =========================================================
    // Sample Pickup Poses
    // =========================================================
    
    // Sample 1 (Rightmost/First one)
    // Score Point -> Intermediate -> Pickup
    public static final Pose BLUE_SAMPLE_1_INTERMEDIATE = new Pose(42.071, 83.694, toRadians(180));
    public static final Pose RED_SAMPLE_1_INTERMEDIATE = new Pose(101.929, 83.694, toRadians(0));

    // Sample 1 Pose: x: 23.785, y: 83.87, endDeg: 180
    public static final Pose BLUE_SAMPLE_1_POSE = new Pose(23.785, 83.87, toRadians(180));
    public static final Pose RED_SAMPLE_1_POSE = new Pose(120.215, 83.87, toRadians(0));

    // Sample 2 (Middle one)
    // Path 4 leads to an intermediate point, Path 5 leads to the pickup
    public static final Pose BLUE_SAMPLE_2_INTERMEDIATE = new Pose(42.492, 59.952, toRadians(180));
    public static final Pose RED_SAMPLE_2_INTERMEDIATE = new Pose(101.508, 59.952, toRadians(0));
    
    public static final Pose BLUE_SAMPLE_2_POSE = new Pose(24.052, 59.819, toRadians(180));
    public static final Pose RED_SAMPLE_2_POSE = new Pose(119.948, 59.819, toRadians(0));

    // Sample 3 (Leftmost/Last one)
    // Path 7 leads to an intermediate point, Path 8 leads to the pickup
    public static final Pose BLUE_SAMPLE_3_INTERMEDIATE = new Pose(42.091, 36.034, toRadians(180));
    public static final Pose RED_SAMPLE_3_INTERMEDIATE = new Pose(101.909, 36.034, toRadians(0));
    
    public static final Pose BLUE_SAMPLE_3_POSE = new Pose(23.918, 35.633, toRadians(180));
    public static final Pose RED_SAMPLE_3_POSE = new Pose(120.082, 35.633, toRadians(0));

    // =========================================================
    // Gate Poses
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
