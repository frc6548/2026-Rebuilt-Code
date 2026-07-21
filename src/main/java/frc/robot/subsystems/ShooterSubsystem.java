package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooter1 = new TalonFX(10);
    private final TalonFX shooter2 = new TalonFX(14);
    private final VoltageOut voltageOut = new VoltageOut(0);

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Voltage compensation — maintains consistent output despite battery sag
        config.Voltage.PeakForwardVoltage  = 11.5;
        config.Voltage.PeakReverseVoltage  = -11.5;

        // Ramp up over ~2 seconds — matches voltage compensation mode
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2.0;

        // Brake mode — stops cleanly when commanded off
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits to protect Krakens during spin-up
        config.CurrentLimits.StatorCurrentLimit             = 80;
        config.CurrentLimits.StatorCurrentLimitEnable       = true;
        config.CurrentLimits.SupplyCurrentLimit             = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable       = true;
        config.CurrentLimits.SupplyCurrentLowerLimit        = 40;  // sustained current limit
        config.CurrentLimits.SupplyCurrentLowerTime         = 1.0; // after 1 second drop to 40A

        shooter1.getConfigurator().apply(config);
        shooter2.getConfigurator().apply(config);
    }

    public void spin() {
        shooter1.setControl(voltageOut.withOutput(-11.5 * 0.45));
        shooter2.setControl(voltageOut.withOutput( 11.5 * 0.45));
    }

    public void stop() {
        shooter1.setControl(voltageOut.withOutput(0));
        shooter2.setControl(voltageOut.withOutput(0));
    }

    public double getVelocityRPS() {
        return shooter1.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter1 Velocity (RPS)",      shooter1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter2 Velocity (RPS)",      shooter2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter1 Motor Output",        shooter1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter2 Motor Output",        shooter2.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter1 Stator Current",      shooter1.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter2 Stator Current",      shooter2.getStatorCurrent().getValueAsDouble());
    }
}