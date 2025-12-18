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
     * Reads gamepad inputs, applies deadband, and sends field-centric drive commands.
     * Active braking is enabled via setGamepad() when inputs are below deadband.
     */
    @Override
    public void execute() {
        if (!isAuto[0]) {
            // check for input outside deadband
            if (Math.abs(gamepadEx.getLeftX()) > DriveConstants.deadband || Math.abs(gamepadEx.getLeftY()) > DriveConstants.deadband || Math.abs(gamepadEx.getRightX()) > DriveConstants.deadband) {
                drive.setGamepad(true); // Signal that gamepad is active (disables automatic braking if implemented)
                
                // Drive Field Relative
                // Note: Left Y is inverted (-gamepadEx.getLeftY()) because gamepad Y is negative up.
                // Left X is Strafe. Right X is Turn.
                drive.moveRobotFieldRelative(-gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX());
            }
            else {
                drive.setGamepad(false); // Signal that gamepad is inactive (enables braking)
            }
        }
    }
}
