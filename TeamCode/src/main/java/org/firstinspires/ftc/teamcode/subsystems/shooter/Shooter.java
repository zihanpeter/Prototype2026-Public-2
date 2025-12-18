package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.releaseVelocity;
import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.shooterServoUpPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.Util;

/**
 * Subsystem handling the Shooter mechanism.
 * Controls the shooter flywheels and the angle adjustment servo.
 */
public class Shooter extends SubsystemBase {
    public final DcMotorEx rightShooter;
    public final DcMotorEx leftShooter;
    public final Servo shooterServo;
    public final TelemetryPacket packet = new TelemetryPacket();
    
    // Flag indicating if the shooter is up to speed
    public static boolean readyToShoot = false;
    
    // PID Controller (Currently unused, replaced by Bang-Bang/Feedforward)
    public final PIDController pidController;
    
    // Current state of the shooter
    public ShooterState shooterState = ShooterState.STOP;

    /**
     * Constructor for Shooter.
     * Initializes motors, servo, and PID controller.
     *
     * @param hardwareMap The hardware map.
     */
    public Shooter(final HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        shooterServo = hardwareMap.get(Servo.class, ShooterConstants.shooterServoName);
        pidController = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
    }

    /**
     * Enum representing the various states of the shooter.
     * Each state defines a target velocity and a servo position.
     */
    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity, ShooterConstants.shooterServoDownPos),
        SLOW(ShooterConstants.slowVelocity, ShooterConstants.shooterServoDownPos),
        MID(ShooterConstants.midVelocity, ShooterConstants.shooterServoMidPos),
        FAST(ShooterConstants.fastVelocity, ShooterConstants.shooterServoUpPos);

        final double shooterVelocity, shooterServoPos;

        ShooterState(double shooterVelocity, double shooterServoPos) {
            this.shooterVelocity = shooterVelocity;
            this.shooterServoPos = shooterServoPos;
        }
    }

    /**
     * Sets the target state for the shooter.
     * @param shooterState The desired ShooterState.
     */
    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    /**
     * Gets the current velocity of the right shooter motor.
     * @return Velocity in ticks per second.
     */
    public double getVelocity() {
        return rightShooter.getVelocity();
    }

    /**
     * Gets the target velocity based on the current state.
     * @return Target velocity in ticks per second.
     */
    public double getTargetVelocity() {
        return shooterState.shooterVelocity;
    }

    /**
     * Checks if the shooter has reached its target velocity.
     * @return True if at or above (more negative) target speed.
     */
    public boolean isShooterAtSetPoint() {
        // Since velocity is negative, we check if current velocity is less than or equal to target velocity 
        // (meaning magnitude is greater than or equal to target magnitude)
        return rightShooter.getVelocity() <= shooterState.shooterVelocity;
    }

    /**
     * Periodic update method.
     * Implements Bang-Bang control with Feedforward for velocity regulation.
     */
    @Override
    public void periodic() {
        // Control loop runs always (even in STOP state) to maintain idle speed if set
        double currentVel = rightShooter.getVelocity();
        double targetVel = shooterState.shooterVelocity;
        double power;

        // Bang-Bang Control with Simple Feedforward Logic
        // Note: Velocities are negative (e.g., Target: -1500)
        // currentVel > targetVel (e.g. -1000 > -1500) means we are SLOWER (less negative magnitude) -> Need MAX power to accelerate
        // currentVel <= targetVel (e.g. -2000 <= -1500) means we are FASTER (more negative magnitude) -> Need FEEDFORWARD power to maintain
        
        if (currentVel > targetVel) {
            // Too slow, apply max power to accelerate
            power = 1.0;
        } else {
            // Too fast or at speed, reduce power to feedforward value to maintain speed
            // Estimate based on GoBilda 5203-2402-0001 (6000RPM)
            // Max TPS = (6000 / 60) * 28 = 2800 TPS
            // Ratio = |target| / 2800.0
            power = Math.abs(targetVel) / ShooterConstants.maxVelocityTPS; 
        }

        // Apply power
        // leftShooter runs positive, rightShooter runs negative
        leftShooter.setPower(power);
        rightShooter.setPower(-power);

        // Update Servo Position
        shooterServo.setPosition(shooterState.shooterServoPos);

        // Update Ready Flag
        if (isShooterAtSetPoint()) {
            readyToShoot = true;
        } else if (rightShooter.getVelocity() > releaseVelocity) {
            // Reset ready flag if speed drops significantly (closer to 0 than releaseVelocity)
            readyToShoot = false;
        }

        // Telemetry handled centrally
        // packet.put("rightShooterVelocity", rightShooter.getVelocity());
        // FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
