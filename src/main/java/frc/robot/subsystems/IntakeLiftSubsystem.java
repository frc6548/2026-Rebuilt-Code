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

    // Positions — UP is 0, DOWN is 48.09 (125:1 total gear ratio — three 5:1 stages)
    public static final double POSITION_UP   =  0.0;
    public static final double POSITION_DOWN =  48.09521484375;
    public static final double DUMP          =  17.0;  // ~35% down, tune as needed
    public static final double ZERO          =  0.0;   // Explicit reset alias for UP

    private double heldPosition = POSITION_UP; // start held at UP on boot

    public IntakeLiftSubsystem() {
        intakeLift = new TalonFX(12);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID & feedforward
        config.Slot0.kP = 4.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.3;
        config.Slot0.kS = 0.5;
        config.Slot0.kV = 0.12;
        config.Slot0.kG = 0.0;   // Removed — was causing initial jerk fighting gravity going DOWN

        // MotionMagic profile — smoother motion over 48 rotations
        config.MotionMagic.MotionMagicCruiseVelocity = 35;  // slightly lower cruise for smoother run
        config.MotionMagic.MotionMagicAcceleration   = 30;  // slower ramp up/down = no jerky pauses
        config.MotionMagic.MotionMagicJerk           = 100; // low jerk = very smooth transitions

        // Brake mode — holds position when not commanded
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;

        // Software limits — just outside real range
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50.0;  // buffer past DOWN
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;  // buffer past UP

        // Current limit to protect the motors
        config.CurrentLimits.StatorCurrentLimit       = 80;  // raised from 60
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeLift.getConfigurator().apply(config);
        intakeLift.setPosition(0.0); // Zero on boot — make sure intake is UP when powering on
    }

    public void setPosition(double rotations) {
        intakeLift.setControl(positionRequest.withPosition(rotations));
    }

    public double getPosition() {
        return intakeLift.getPosition().getValueAsDouble();
    }

    /** Call once when a command ends to snapshot the current position for holdPosition(). */
    public void snapshotPosition() {
        heldPosition = getPosition();
    }

    /** Actively holds the last snapshotted position using MotionMagic. */
    public void holdPosition() {
        intakeLift.setControl(positionRequest.withPosition(heldPosition));
    }

    /** Full stop — use only when you want to release hold (e.g. disabled). */
    public void stop() {
        intakeLift.stopMotor();
    }

    public boolean atTarget(double targetRotations) {
        return Math.abs(getPosition() - targetRotations) < 0.5; // wider tolerance for 125:1
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Lift Position",       getPosition());
        SmartDashboard.putNumber("Intake Lift Held Target",    heldPosition);
        SmartDashboard.putNumber("Intake Lift Output",         intakeLift.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake Lift Error",          intakeLift.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Intake Lift Stator Current", intakeLift.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putString("Intake Lift Rev Limit",      intakeLift.getReverseLimit().getValue().toString());
        SmartDashboard.putString("Intake Lift Active Command",
            getCurrentCommand() != null ? getCurrentCommand().getName() : "Default");
    }
}