package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    // Adjust these positions (in rotations) to match your hood's mechanics
    public static final double Hardstop = 0;
    public static final double TestPosition = 0.2;// 0.06103515625

    private final TalonFX hoodMotor;

    public HoodSubsystem(int motorCanId) {
    hoodMotor = new TalonFX(motorCanId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        
    config.Slot0.kP = 2.0;   // Much gentler for 0.2 rotation range
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.1;
    config.Slot0.kS = 0.3;
    config.Slot0.kV = 0.12;
    config.Slot0.kG = 0.2;

    config.MotionMagic.MotionMagicCruiseVelocity = 5;   // Very slow — 5 rot/sec
    config.MotionMagic.MotionMagicAcceleration   = 10;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 11.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit       = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Motion magic for smooth movement (optional but recommended)
        config.MotionMagic.MotionMagicCruiseVelocity = 80;
        config.MotionMagic.MotionMagicAcceleration   = 160;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // Software limits to protect the hood
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable   = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold  = 11.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable   = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;

        hoodMotor.getConfigurator().apply(config);
        hoodMotor.setPosition(0.0);
    }

    public void setPosition(double rotations) {
        hoodMotor.setControl(positionRequest.withPosition(rotations));
    }

    public double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public boolean atTarget(double targetRotations) {
        return Math.abs(getPosition() - targetRotations) < 0.02; // 0.1 rotation tolerance
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position", getPosition()); // Useful for tuning
        SmartDashboard.putNumber("Hood Motor Output", hoodMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood Closed Loop Error", hoodMotor.getClosedLoopError().getValueAsDouble());
    }
}