package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivePinpoint;

/**
 * Command for TeleOp driving.
 * Handles gamepad input and sends movement commands to the drive subsystem.
 */
public class TeleOpDriveCommand extends CommandBase {
    private final MecanumDrivePinpoint drive;
    private final GamepadEx gamepadEx;
    private final boolean[] isAuto;

    /**
     * Constructor for TeleOpDriveCommand.
     *
     * @param drive The drive subsystem.
     * @param gamepadEx The gamepad input wrapper.
     * @param isAuto Flag array to indicate if autonomous mode is active (stops manual control).
     */
    public TeleOpDriveCommand(MecanumDrivePinpoint drive, GamepadEx gamepadEx, boolean[] isAuto) {
        this.drive = drive;
        this.gamepadEx = gamepadEx;
        this.isAuto = isAuto;
        addRequirements(drive);
    }

    /**
     * Executes the drive logic.
     * Reads gamepad inputs, applies deadband and squared input curve, and sends field-centric drive commands.
     * Active braking is enabled via setGamepad() when inputs are below deadband.
     * D-Pad Left/Right can also be used for rotation.
     */
    @Override
    public void execute() {
        if (!isAuto[0]) {
            // Get raw inputs
            double rawLeftX = gamepadEx.getLeftX();
            double rawLeftY = gamepadEx.getLeftY();
            double rawRightX = gamepadEx.getRightX();
            
            // D-Pad rotation input
            double dpadTurn = 0;
            if (gamepadEx.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT)) {
                dpadTurn = -DriveConstants.dpadTurnSpeed; // Turn left (negative)
            } else if (gamepadEx.getButton(com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT)) {
                dpadTurn = DriveConstants.dpadTurnSpeed;  // Turn right (positive)
            }
            
            // check for input outside deadband OR D-Pad is pressed
            boolean hasInput = Math.abs(rawLeftX) > DriveConstants.deadband || 
                               Math.abs(rawLeftY) > DriveConstants.deadband || 
                               Math.abs(rawRightX) > DriveConstants.deadband ||
                               dpadTurn != 0;
            
            if (hasInput) {
                drive.setGamepad(true); // Signal that gamepad is active (disables automatic braking if implemented)
                
                // Apply squared input curve while preserving sign
                // Formula: squared = value * |value| (preserves sign, squares magnitude)
                double forward = rawLeftY * Math.abs(rawLeftY);  // Inverted because gamepad Y is negative up
                double strafe = -rawLeftX * Math.abs(rawLeftX);
                double turn = rawRightX * Math.abs(rawRightX);   // Inverted based on user request
                
                // Add D-Pad turn input
                turn += dpadTurn;
                
                // Drive Field Relative
                drive.moveRobotFieldRelative(forward, strafe, turn);
            }
            else {
                drive.setGamepad(false); // Signal that gamepad is inactive (enables braking)
            }
        }
    }
}
