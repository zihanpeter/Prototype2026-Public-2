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

public class Shooter extends SubsystemBase {
    public final DcMotorEx rightShooter;
    public final DcMotorEx leftShooter;
    public final Servo shooterServo;
    public final TelemetryPacket packet = new TelemetryPacket();
    public static boolean readyToShoot = false;
    public final PIDController pidController;
    public ShooterState shooterState = ShooterState.STOP;

    public Shooter(final HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterName);
        leftShooter = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterName);
        shooterServo = hardwareMap.get(Servo.class, ShooterConstants.shooterServoName);
        pidController = new PIDController(ShooterConstants.kP,
                ShooterConstants.kI, ShooterConstants.kD);
    }

    public enum ShooterState {
        STOP(ShooterConstants.stopVelocity, ShooterConstants.shooterServoDownPos),
        SLOW(ShooterConstants.slowVelocity, ShooterConstants.shooterServoDownPos),
        FAST(ShooterConstants.fastVelocity, ShooterConstants.shooterServoUpPos);

        final double shooterVelocity, shooterServoPos;

        ShooterState(double shooterVelocity, double shooterServoPos) {
            this.shooterVelocity = shooterVelocity;
            this.shooterServoPos = shooterServoPos;
        }
    }


    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public double getVelocity() {
        return rightShooter.getVelocity();
    }

    public boolean isShooterAtSetPoint() {
        return Util.epsilonEqual(shooterState.shooterVelocity,
                rightShooter.getVelocity(), ShooterConstants.shooterEpsilon);
    }

    @Override
    public void periodic() {
        if (shooterState != ShooterState.STOP) {
            leftShooter.setPower(1);
            rightShooter.setPower(-1);
        }
        else {
            leftShooter.setPower(ShooterState.STOP.shooterVelocity);
            rightShooter.setPower(-ShooterState.STOP.shooterVelocity);
        }

        shooterServo.setPosition(shooterState.shooterServoPos);

        if (isShooterAtSetPoint()) {
            readyToShoot = true;
        }
        else if (rightShooter.getVelocity() <= releaseVelocity) {
            readyToShoot = false;
        }

        packet.put("rightShooterVelocity", rightShooter.getVelocity());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
