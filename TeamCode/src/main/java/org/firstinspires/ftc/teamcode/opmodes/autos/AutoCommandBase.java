package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drive.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

/**
 * Base class for Autonomous OpModes.
 * Handles initialization of subsystems, telemetry, and the command scheduler loop.
 */
public abstract class AutoCommandBase extends LinearOpMode {
    protected Shooter shooter;
    protected Transit transit;
    protected Intake intake;
    protected Follower follower;

    /**
     * Abstract method to define the autonomous command sequence.
     * @return The Command to run.
     */
    public abstract Command runAutoCommand();

    /**
     * Abstract method to define the starting pose of the robot.
     * @return The starting Pose.
     */
    public abstract Pose getStartPose();

    /**
     * Initializes subsystems and telemetry.
     */
    private void initialize() {
        // Setup MultipleTelemetry to display on both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Follower with hardware map
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(getStartPose());

        // Initialize subsystems
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // Get the command sequence defined in the child class
        Command toRun = runAutoCommand();

        // Schedule the command
        CommandScheduler.getInstance().schedule(toRun);

        waitForStart();

        // Main Loop
        while (opModeIsActive() && !isStopRequested()) {
            // Run the CommandScheduler to execute scheduled commands
            CommandScheduler.getInstance().run();
            
            // Update telemetry
            telemetry.update();
        }

        onAutoStopped();
        CommandScheduler.getInstance().reset();
    }

    /**
     * Executes when auto is stopped. Can be overridden for cleanup.
     */
    public void onAutoStopped() {}
}
