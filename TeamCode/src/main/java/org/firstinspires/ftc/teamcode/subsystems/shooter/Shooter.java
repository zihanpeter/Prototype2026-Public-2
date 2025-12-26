package org.firstinspires.ftc.teamcode.subsystems.shooter;

import static org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.releaseVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public final Servo brakeServo;
    public final TelemetryPacket packet = new TelemetryPacket();
    
    // Flag indicating if the shooter is up to speed
//    public static boolean readyToShoot = false;
    
    // Flag indicating if the brake is engaged
    public static boolean brakeEngaged = false;
    
    // Flag for manual brake override (takes priority over auto brake)
    public static boolean manualBrakeOverride = false;
    
    // Auto brake cycle: enabled when shoot button released, disabled once speed reaches stopVelocity
    private boolean autoBrakeCycleActive = false;
    
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
        brakeServo = hardwareMap.get(Servo.class, ShooterConstants.brakeServoName);
        pidController = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
        
        // Initialize brake to released position
        brakeServo.setPosition(ShooterConstants.brakeServoReleasedPos);
        brakeEngaged = false;
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
        this.adaptiveVelocity = 0;  // Clear adaptive velocity when using state
        this.adaptiveServoPosition = -1;  // Clear adaptive servo position when using state
    }
    
    // Adaptive velocity for auto-fire (0 means use state velocity)
    private double adaptiveVelocity = 0;
    
    /**
     * Sets an adaptive velocity for auto-fire.
     * This takes priority over the state velocity.
     * @param velocity Target velocity in TPS (negative value)
     */
    public void setAdaptiveVelocity(double velocity) {
        this.adaptiveVelocity = velocity;
    }
    
    /**
     * Gets the current adaptive velocity setting.
     */
    public double getAdaptiveVelocity() {
        return adaptiveVelocity;
    }
    
    // Adaptive servo position for auto-fire (-1 means use state servo position)
    private double adaptiveServoPosition = -1;
    
    /**
     * Sets an adaptive servo position for auto-fire.
     * This takes priority over the state servo position.
     * @param position Servo position (0-1), or -1 to use state position
     */
    public void setAdaptiveServoPosition(double position) {
        this.adaptiveServoPosition = position;
    }
    
    /**
     * Gets the current adaptive servo position setting.
     */
    public double getAdaptiveServoPosition() {
        return adaptiveServoPosition;
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
     * Considers adaptiveVelocity if set.
     * @return True if at or above (more negative) target speed.
     */
    public boolean isShooterAtSetPoint() {
        // Determine target velocity: use adaptive if set, otherwise use state velocity
        double targetVel = (adaptiveVelocity != 0) ? adaptiveVelocity : shooterState.shooterVelocity;
        
        // If using state velocity and state is STOP, return false
        if (adaptiveVelocity == 0 && shooterState == ShooterState.STOP) {
            return false;
        }
        
        // Check if current velocity is close to target velocity
        return Util.epsilonEqual(
                rightShooter.getVelocity(),
                targetVel,
                ShooterConstants.shooterEpsilon
        );
    }

    /**
     * Engages the brake servo to stop the flywheel.
     */
    public void engageBrake() {
        brakeServo.setPosition(ShooterConstants.brakeServoEngagedPos);
        brakeEngaged = true;
    }

    /**
     * Releases the brake servo to allow the flywheel to spin.
     */
    public void releaseBrake() {
        brakeServo.setPosition(ShooterConstants.brakeServoReleasedPos);
        brakeEngaged = false;
    }

    /**
     * Toggles the brake servo state.
     */
    public void toggleBrake() {
        if (brakeEngaged) {
            releaseBrake();
        } else {
            engageBrake();
        }
    }

    /**
     * Checks if the brake is currently engaged.
     * @return True if brake is engaged.
     */
    public boolean isBrakeEngaged() {
        return brakeEngaged;
    }

    /**
     * Manually engages the brake (overrides auto brake logic).
     */
    public void manualEngageBrake() {
        manualBrakeOverride = true;
        engageBrake();
    }

    /**
     * Manually releases the brake (returns control to auto brake logic).
     */
    public void manualReleaseBrake() {
        manualBrakeOverride = false;
        releaseBrake();
    }
    
    /**
     * Starts an auto brake cycle.
     * Called when shoot button (LB/RB/RT) is released.
     * Brake will stay engaged until speed drops to stopVelocity.
     */
    public void startAutoBrakeCycle() {
        autoBrakeCycleActive = true;
    }
    
    /**
     * Cancels the auto brake cycle.
     * Called when shoot button (LB/RB/RT) is pressed.
     */
    public void cancelAutoBrakeCycle() {
        autoBrakeCycleActive = false;
        if (!manualBrakeOverride) {
            releaseBrake();
        }
    }
    
    /**
     * Checks if auto brake cycle is currently active.
     */
    public boolean isAutoBrakeCycleActive() {
        return autoBrakeCycleActive;
    }

//    public void setReadyToShoot(boolean ready) {
//        readyToShoot = ready;
//    }
//
//    public boolean getReadyToShoot() {
//        return readyToShoot;
//    }

    /**
     * Periodic update method.
     * Implements Bang-Bang control with Feedforward for velocity regulation.
     * STOP state uses open-loop idle power (no PID).
     */
    @Override
    public void periodic() {
        // Control loop runs always (even in STOP state) to maintain idle speed if set
        double currentVel = rightShooter.getVelocity();
        
        // Use adaptive velocity if set, otherwise use state velocity
        double targetVel = (adaptiveVelocity != 0) ? adaptiveVelocity : shooterState.shooterVelocity;
        double power;

        // =================================================================
        // Auto Brake Logic
        // Triggered when shoot button (LB/RB/RT) is released.
        // Brake stays engaged until speed drops below brakeReleaseThresholdTPS (-680 TPS).
        // Once reached, brake is released and cycle ends.
        // Skip auto logic if manual override is active.
        // =================================================================
        if (!manualBrakeOverride && autoBrakeCycleActive) {
            // Check if speed has dropped below threshold
            // Velocities are negative: -680 is threshold
            // currentVel >= threshold means we've slowed down enough (e.g., -500 >= -680)
            if (currentVel >= ShooterConstants.brakeReleaseThresholdTPS) {
                // Speed has dropped below threshold, end brake cycle
                autoBrakeCycleActive = false;
                releaseBrake();
            } else {
                // Still decelerating, keep brake engaged
                if (!brakeEngaged) {
                    engageBrake();
                }
            }
        }
        // If manualBrakeOverride is true, brake state is controlled manually

        // =================================================================
        // Motor Power Control
        // STOP state (without adaptive velocity): Open-loop idle power (0.27), no PID control
        // Other states or adaptive velocity: Bang-Bang control with feedforward
        // =================================================================
        if (shooterState == ShooterState.STOP && adaptiveVelocity == 0) {
            // Idle mode: Use fixed open-loop power, no closed-loop control
            power = ShooterConstants.idlePower;
        } else {
            // Bang-Bang Control with Simple Feedforward Logic
            // Note: Velocities are negative (e.g., Target: -1500)
            // currentVel > targetVel (e.g. -1000 > -1500) means we are SLOWER -> Need MAX power to accelerate
            // currentVel <= targetVel (e.g. -2000 <= -1500) means we are FASTER -> Need FEEDFORWARD power to maintain
            
            if (currentVel > targetVel) {
                // Too slow, apply max power to accelerate
                power = 1.0;
            } else {
                // Too fast or at speed, reduce power to feedforward value to maintain speed
                // Ratio = |target| / maxVelocityTPS
                power = Math.abs(targetVel) / ShooterConstants.maxVelocityTPS;
            }
        }

        // Apply power
        // leftShooter runs positive, rightShooter runs negative
        leftShooter.setPower(power);
        rightShooter.setPower(-power);

        // Update Servo Position
        // Use adaptive servo position if set, otherwise use state servo position
        double servoPos = (adaptiveServoPosition >= 0) ? adaptiveServoPosition : shooterState.shooterServoPos;
        shooterServo.setPosition(servoPos);

        // Telemetry handled centrally
    }
}
