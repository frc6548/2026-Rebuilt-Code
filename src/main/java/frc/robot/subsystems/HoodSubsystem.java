package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    private final StaticBrake brakeRequest = new StaticBrake();

    public static final double Hardstop     = 0;
    public static final double TestPosition = 2.32421875;

    private final TalonFX hoodMotor;

    public HoodSubsystem(int motorCanId) {
        hoodMotor = new TalonFX(motorCanId);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 10.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kS = 0.3;
        config.Slot0.kV = 0.12;
        config.Slot0.kG = .2;

        config.MotionMagic.MotionMagicCruiseVelocity = 80;
        config.MotionMagic.MotionMagicAcceleration   = 160;

        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 11.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;

        config.CurrentLimits.StatorCurrentLimit       = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        hoodMotor.getConfigurator().apply(config);
        hoodMotor.setPosition(0.0);

    // Read back the config to verify it was applied
    TalonFXConfiguration readback = new TalonFXConfiguration();
    hoodMotor.getConfigurator().refresh(readback);
    System.out.println(">>> Hood NeutralMode: " + readback.MotorOutput.NeutralMode);
    
    var status = hoodMotor.getConfigurator().apply(config);
    System.out.println(">>> Config apply status: " + status);

hoodMotor.getConfigurator().refresh(readback);
System.out.println(">>> kP readback: " + readback.Slot0.kP);
System.out.println(">>> Config status: " + hoodMotor.getConfigurator().apply(config));
    }

    public void setPosition(double rotations) {
        hoodMotor.setControl(positionRequest.withPosition(rotations));
    }

    public double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    private final NeutralOut neutralRequest = new NeutralOut();

    public void stop() {
     hoodMotor.setControl(brakeRequest);
    }

    public boolean atTarget(double targetRotations) {
        return Math.abs(getPosition() - targetRotations) < 0.1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position", getPosition());
        SmartDashboard.putNumber("Hood Motor Output", hoodMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood Closed Loop Error", hoodMotor.getClosedLoopError().getValueAsDouble());
    }
}