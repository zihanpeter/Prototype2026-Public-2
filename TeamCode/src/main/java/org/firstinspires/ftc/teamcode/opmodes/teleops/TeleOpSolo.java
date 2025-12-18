package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.controls.DriverControls;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

/**
 * Main TeleOp OpMode for Solo operation.
 * Field-Centric Driving with various subsystems integration.
 */
@Config
@Configurable
@TeleOp(name = "TeleOp Solo")
public class TeleOpSolo extends CommandOpMode {
    // Robot Container
    private Robot robot;

    // Gamepad
    private GamepadEx gamepadEx1;

    // Flags
    private boolean[] isAuto = {false};

    @Override
    public void initialize() {
        // Initialize Robot Hardware
        robot = new Robot(hardwareMap);
        
        // Initialize Gamepad Wrapper
        gamepadEx1 = new GamepadEx(gamepad1);

        // Set Default Drive Command (Field Centric)
        robot.drive.setDefaultCommand(new TeleOpDriveCommand(robot.drive, gamepadEx1, isAuto));

        // --- Driver Controls ---
        // Bind gamepad controls using the centralized DriverControls class
        DriverControls.bind(gamepadEx1, robot);

        // Setup Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        // Run Command Scheduler
        CommandScheduler.getInstance().run();

        // Get Current Pose
        Pose2D pose = robot.drive.getPose();

        // --- Driver Station Telemetry ---
        telemetry.addData("X", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y",  pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", pose.getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset", robot.drive.getYawOffset());
        telemetry.addData("Shooter Current Velocity", robot.shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity", robot.shooter.getTargetVelocity());
        telemetry.addData("Intake Velocity", robot.intake.getVelocity());

        // Check and display shooter status
        if (robot.shooter.isShooterAtSetPoint() && robot.shooter.getTargetVelocity() != 0) {
            telemetry.addData("Shooter Status", "TARGET SPEED REACHED");
        }
        
        // Debug inputs
        telemetry.addData("Gamepad Lx: ", gamepadEx1.getLeftX());
        telemetry.addData("Gamepad Ly: ", gamepadEx1.getLeftY());
        telemetry.addData("Gamepad Rx: ", gamepadEx1.getRightX());
        
        // Debug Motor Powers
        telemetry.addData("LF Power: ", robot.drive.leftBackMotor.getPower());
        telemetry.addData("RF Power: ", robot.drive.rightFrontMotor.getPower());
        telemetry.addData("LB Power: ", robot.drive.leftBackMotor.getPower());
        telemetry.addData("RB Motor: ", robot.drive.rightBackMotor.getPower());
        telemetry.addData("Is Gamepad On: ", robot.drive.isGamepadOn);
        
        telemetry.update();

        // --- Dashboard Telemetry Packet ---
        // Send a single consolidated packet to Dashboard for graphing and Field Drawing
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ShooterVelocity", robot.shooter.getVelocity());
        packet.put("IntakeVelocity", robot.intake.getVelocity());
        // Determine target intake power for graph
        double targetIntakePower = Intake.isShooting ? 
                org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.transitPower : 
                org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.intakePower;
        packet.put("IntakeTargetPower", targetIntakePower);
        
        // Draw Robot on Dashboard Field
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5"); // Blue color
        // Draw a circle representing the robot (Radius approx 9 inches)
        fieldOverlay.strokeCircle(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), 9);
        // Draw a line indicating heading
        double heading = pose.getHeading(AngleUnit.RADIANS);
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        fieldOverlay.strokeLine(x, y, x + 9 * Math.cos(heading), y + 9 * Math.sin(heading));

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
