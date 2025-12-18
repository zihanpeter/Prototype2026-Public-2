package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem handling the Intake mechanism.
 * Controls the intake motor for collecting game elements.
 */
public class Intake extends SubsystemBase {
    public final DcMotorEx intakeMotor;
    public final TelemetryPacket packet = new TelemetryPacket();

    public static boolean isRunning = false;
    public static boolean isShooting = false;
    public static boolean isFullPower = false;
    public static boolean isReversed = false;

    /**
     * Constructor for Intake.
     * Initializes the intake motor and sets default state.
     *
     * @param hardwareMap The hardware map.
     */
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.intakeMotorName);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        isRunning = true; // Intake runs continuously by default after initialization
    }

    /**
     * Starts the intake (sets isRunning to true).
     */
    public void startIntake() {
        isRunning = true;
    }

    /**
     * Stops the intake (sets isRunning to false).
     */
    public void stopIntake() {
        isRunning = false;
    }

    /**
     * Toggles the intake running state (On/Off).
     */
    public void toggle() {
        isRunning = !isRunning;
    }

    /**
     * Checks if the intake is currently running.
     * @return True if running.
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Toggles the shooting state flag.
     * When shooting, intake might need to run at a specific power (transitPower).
     */
    public void toggleShooting() {
        isShooting = !isShooting;
    }

    /**
     * Checks if the intake is in shooting mode.
     * @return True if shooting.
     */
    public boolean isShooting() {
        return isShooting;
    }

    /**
     * Sets the full power flag.
     * Used to override standard intake power with maximum power.
     *
     * @param fullPower True to enable full power.
     */
    public void setFullPower(boolean fullPower) {
        isFullPower = fullPower;
    }

    /**
     * Sets the reversed flag.
     * Used to reverse the intake direction (e.g., for ejecting).
     *
     * @param reversed True to reverse intake.
     */
    public void setReversed(boolean reversed) {
        isReversed = reversed;
    }

    /**
     * Gets the current velocity of the intake motor.
     * @return Velocity in ticks per second.
     */
    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    /**
     * Periodic update method.
     * Controls the motor power based on the current state flags.
     */
    @Override
    public void periodic() {
        if (isRunning) {
            double targetPower = IntakeConstants.intakePower;
            
            // Priority logic for power selection
            if (isShooting) {
                targetPower = IntakeConstants.transitPower;
            } else if (isFullPower) {
                targetPower = IntakeConstants.fullPower;
            }

            // Reverse logic
            if (isReversed) {
                // When reversed, use full power (negative)
                targetPower = -IntakeConstants.fullPower;
            }

            intakeMotor.setPower(targetPower);
        }
        else {
            intakeMotor.setPower(0);
        }
    }
}
