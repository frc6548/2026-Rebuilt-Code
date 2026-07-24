package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX hoodMotor;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);
    private final StaticBrake brakeRequest = new StaticBrake();

    // Positions
    public static final double HARDSTOP = 0.0;
    public static final double MAX      = 3.17431640625;

    private double heldPosition = HARDSTOP; // hold at hardstop on boot

    public HoodSubsystem(int motorCanId) {
        hoodMotor = new TalonFX(motorCanId);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID & feedforward
        config.Slot0.kP = 10.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        config.Slot0.kS = 0.3;
        config.Slot0.kV = 0.12;
        config.Slot0.kG = 0.2;

        // MotionMagic - smooth over short 3.17 rotation range
        config.MotionMagic.MotionMagicCruiseVelocity = 10;
        config.MotionMagic.MotionMagicAcceleration   = 20;
        config.MotionMagic.MotionMagicJerk           = 100;

        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Software limits - just outside real range
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 3.3;   // just past MAX
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;  // just past HARDSTOP

        config.CurrentLimits.StatorCurrentLimit       = 40;  // hood is small, 100A was excessive
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        hoodMotor.getConfigurator().apply(config);
        hoodMotor.setPosition(0.0); // Zero on boot - make sure hood is at hardstop when powering on

        // Single readback to verify
        TalonFXConfiguration readback = new TalonFXConfiguration();
        hoodMotor.getConfigurator().refresh(readback);
        System.out.println(">>> Hood kP readback: " + readback.Slot0.kP);
        System.out.println(">>> Hood NeutralMode: " + readback.MotorOutput.NeutralMode);
    }

    public void setPosition(double rotations) {
        hoodMotor.setControl(positionRequest.withPosition(rotations));
    }

    public double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public void snapshotPosition() {
        heldPosition = getPosition();
    }

    public void resetToDefault() {
        heldPosition = HARDSTOP;
    }

    public void holdPosition() {
        hoodMotor.setControl(positionRequest.withPosition(heldPosition));
    }

    public void stop() {
        hoodMotor.setControl(brakeRequest);
    }

    public void incrementPosition(double delta) {
        hoodMotor.setControl(positionRequest.withPosition(getPosition() + delta));
    }

    public boolean atTarget(double targetRotations) {
        return Math.abs(getPosition() - targetRotations) < 0.05;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position",          getPosition());
        SmartDashboard.putNumber("Hood Motor Output",      hoodMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Hood Closed Loop Error", hoodMotor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Hood Stator Current",    hoodMotor.getStatorCurrent().getValueAsDouble());
    }
}