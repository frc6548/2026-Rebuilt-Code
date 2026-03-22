package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLiftSubsystem extends SubsystemBase {

    private final TalonFX intakeLift;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

    // Set these by reading Elastic after manually moving the intake
    public static final double POSITION_UP   = -4.8310546875;  // Tune this 7.41455078125
    public static final double POSITION_DOWN = 37.39404296875;  // Tune this (5:1 ratio so ~0.6 real rotations) 47.01220703125;
    public static final double Zero = 0;
    public static final double Dump = 15.4658203125;
    
    public IntakeLiftSubsystem() {
        intakeLift = new TalonFX(13);

        TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 4.0;
config.Slot0.kI = 0.0;
config.Slot0.kD = 0.2;
config.Slot0.kS = 0.5;
config.Slot0.kV = 0.12;
config.Slot0.kG = 1;  // High — needed to lift against gravity

config.MotionMagic.MotionMagicCruiseVelocity = 80;  // Was 15 — rotations per second
config.MotionMagic.MotionMagicAcceleration   = 160; // Was 25 — rotations per second squared
config.MotionMagic.MotionMagicJerk           = 500; // Was 100 — smooths the ramp

        // Brake mode — holds position when not commanded
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Inversion — flip if intake moves wrong direction
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Software limits — set just outside your real up/down range
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 48;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;

        // Current limit to protect the X44
        config.CurrentLimits.StatorCurrentLimit       = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeLift.getConfigurator().apply(config);
        intakeLift.setPosition(0.0); // Zero on boot — make sure intake is UP when powering on
    }

    public void setPosition(double rotations) {
        intakeLift.setControl(positionRequest.withPosition(rotations));
    }

    public double getPosition() {
        return intakeLift.getPosition().getValueAsDouble();
    }

private double heldPosition = 0.0; // tracks where to hold

public void snapshotPosition() {
    heldPosition = getPosition(); // call this once when command ends
}

public void holdPosition() {
    intakeLift.setControl(positionRequest.withPosition(heldPosition));
}
public void stop() {
    intakeLift.stopMotor();
}

    public boolean atTarget(double targetRotations) {
        return Math.abs(getPosition() - targetRotations) < 0.15;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Lift Position", getPosition());
        SmartDashboard.putNumber("Intake Lift Output", intakeLift.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake Lift Error", intakeLift.getClosedLoopError().getValueAsDouble());
    }
}