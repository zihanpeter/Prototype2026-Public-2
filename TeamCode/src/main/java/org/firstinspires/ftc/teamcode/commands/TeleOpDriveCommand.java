package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;

import java.util.function.BooleanSupplier;

/**
 * Command for TeleOp driving with auto-aim support.
 * Handles gamepad input and sends movement commands to the drive subsystem.
 * When auto-aim is enabled (via isAlign supplier), the turn input is replaced by auto-aim calculation.
 */
public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrivePinpoint drive;
    private final GamepadEx gamepadEx;
    private final boolean[] isAuto;
    private final BooleanSupplier isAlign;

    /**
     * Constructor for TeleOpDriveCommand.
     *
     * @param drive The drive subsystem.
     * @param gamepadEx The gamepad input wrapper.
     * @param isAuto Flag array to indicate if autonomous mode is active (stops manual control).
     * @param isAlign Supplier that returns true when auto-aim should be active (e.g., when A button is pressed).
     */
    public TeleOpDriveCommand(MecanumDrivePinpoint drive, GamepadEx gamepadEx, boolean[] isAuto, BooleanSupplier isAlign) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto;
        this.isAlign = isAlign;
        addRequirements(drive);
    }

    /**
     * Executes the drive logic with optional auto-aim.
     * When auto-aim is active, the turn input is replaced by auto-aim calculation.
     * Otherwise, behaves like manual drive.
     */
    @Override
    public void execute() {
        if (!isAuto[0]) {
            // Get raw inputs
            double rawLeftX = gamepadEx.getLeftX();
            double rawLeftY = gamepadEx.getLeftY();
            double rawRightX = gamepadEx.getRightX();
            
            // D-Pad rotation input (only used when not in align mode)
            double dpadTurn = 0;
            if (!isAlign.getAsBoolean()) {
                if (gamepadEx.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT)) {
                    dpadTurn = -DriveConstants.dpadTurnSpeed; // Turn left (negative)
                } else if (gamepadEx.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT)) {
                    dpadTurn = DriveConstants.dpadTurnSpeed;  // Turn right (positive)
                }
            }
            
            // Check for input outside deadband OR D-Pad is pressed (when not aligning)
            boolean hasInput = Math.abs(rawLeftX) > DriveConstants.deadband || 
                               Math.abs(rawLeftY) > DriveConstants.deadband || 
                               Math.abs(rawRightX) > DriveConstants.deadband ||
                               (dpadTurn != 0 && !isAlign.getAsBoolean()) ||
                               isAlign.getAsBoolean(); // Auto-aim mode counts as input
            
            if (hasInput) {
                drive.setGamepad(true); // Signal that gamepad is active
                
                // Apply squared input curve while preserving sign
                double forward = rawLeftY * Math.abs(rawLeftY);  // Inverted because gamepad Y is negative up
                double strafe = -rawLeftX * Math.abs(rawLeftX);
                
                // Determine turn input: use auto-aim if enabled, otherwise use manual input
                double turn;
                if (isAlign.getAsBoolean()) {
                    // TODO: Replace with actual auto-aim calculation
                    // turn = drive.getAlignTurnPower();
                    turn = 0; // Placeholder until auto-aim is implemented
                } else {
                    turn = rawRightX * Math.abs(rawRightX);
                    turn += dpadTurn;
                }
                
                // Drive Field Relative
                drive.moveRobotFieldRelative(forward, strafe, turn);
            }
            else {
                drive.setGamepad(false); // Signal that gamepad is inactive (enables braking)
            }
        }
    }
}
