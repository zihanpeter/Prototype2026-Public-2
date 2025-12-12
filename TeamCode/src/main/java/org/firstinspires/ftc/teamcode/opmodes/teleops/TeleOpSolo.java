package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TransitCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveOTOS;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.FunctionalButton;

@Config
@Configurable
@TeleOp(name = "TeleOp Solo")
public class TeleOpSolo extends CommandOpMode {
    private MecanumDriveOTOS drive;

    private GamepadEx gamepadEx1;

    private Shooter shooter;

    private Transit transit;

    private Intake intake;

    private boolean[] isAuto = {false};

    @Override
    public void initialize() {
        drive = new MecanumDriveOTOS(hardwareMap);
        gamepadEx1 = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap);
        transit = new Transit(hardwareMap);
        intake = new Intake(hardwareMap);

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepadEx1, isAuto));

        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
        ).whenPressed(
                new InstantCommand(() -> drive.reset(0))
        );

        // Intake (Always running, button toggles it off/on if needed, or maybe reverses?)
        // Per request: "Continuous max power after init".
        // Removing the 'whenHeld' command for Left Trigger since it's now always on.
        // Or remapping Left Trigger to something else (e.g., REVERSE intake / Eject).
        
        // For now, removing the manual IntakeCommand binding on Left Trigger
        // If you want Left Trigger to do something else (like Eject), tell me.
        
        /*
        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5
        ).whenHeld(
                new IntakeCommand(transit, intake)
        );
        */

        // Slow Shoot (Left Trigger)
        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5
        ).whenHeld(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.SLOW))
        ).whenReleased(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // Fast Shoot
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.FAST))
        ).whenReleased(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // Mid Shoot
        new FunctionalButton(
                () -> gamepadEx1.getButton(GamepadKeys.Button.LEFT_BUMPER)
        ).whenHeld(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.MID))
        ).whenReleased(
                new InstantCommand(() -> shooter.setShooterState(Shooter.ShooterState.STOP))
        );

        // Transit
        new FunctionalButton(
                () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5
        ).whenHeld(
                new TransitCommand(transit, intake, shooter)
        );

        // Setup MultipleTelemetry to send data to both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("X", drive.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("Y",  drive.getPose().getY(DistanceUnit.INCH));
        telemetry.addData("Heading", drive.getPose().getHeading(AngleUnit.RADIANS));
        telemetry.addData("YawOffset",drive.getYawOffset());
        telemetry.addData("Shooter Current Velocity", shooter.getVelocity());
        telemetry.addData("Shooter Target Velocity", shooter.getTargetVelocity());
        telemetry.addData("Intake Velocity", intake.getVelocity());
        // telemetry.addData("Intake Velocity", Math.round(intake.getVelocity() / 100.0) * 100.0);

        if (shooter.isShooterAtSetPoint() && shooter.getTargetVelocity() != 0) {
            telemetry.addData("Shooter Status", "TARGET SPEED REACHED");
        }
        
        telemetry.addData("Gamepad Lx: ", gamepadEx1.getLeftX());
        telemetry.addData("Gamepad Ly: ", gamepadEx1.getLeftY());
        telemetry.addData("Gamepad Rx: ", gamepadEx1.getRightX());
        telemetry.addData("LF Power: ", drive.leftBackMotor.getPower());
        telemetry.addData("RF Power: ", drive.rightFrontMotor.getPower());
        telemetry.addData("LB Power: ", drive.leftBackMotor.getPower());
        telemetry.addData("RB Motor: ", drive.rightBackMotor.getPower());
        telemetry.addData("LF Mode: ", drive.leftFrontMotor.getMode());
        telemetry.addData("Is Gamepad On: ", drive.isGamepadOn);
        telemetry.update();

        // Send a single consolidated packet to Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("ShooterVelocity", shooter.getVelocity());
        packet.put("IntakeVelocity", intake.getVelocity());
        packet.put("IntakeTargetPower", Intake.isShooting ? org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.transitPower : org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants.intakePower);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
