package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intake1 = new TalonFX(15);
    private final TalonFX intake2 = new TalonFX(23);
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private boolean running = false;

    public IntakeSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit       = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        intake1.getConfigurator().apply(config);
        intake2.getConfigurator().apply(config);
    }

    public void spin() {
        running = true;
        intake1.setControl(dutyCycle.withOutput(0.65));
        intake2.setControl(dutyCycle.withOutput(-0.65));
    }

    public void stop() {
        running = false;
        intake1.setControl(dutyCycle.withOutput(0));
        intake2.setControl(dutyCycle.withOutput(0));
    }

    public boolean isRunning() {
        return running;
    }

    public double getVelocityRPS() {
        return intake1.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake1 Velocity (RPS)", intake1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake2 Velocity (RPS)", intake2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake1 Motor Output",   intake1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake2 Motor Output",   intake2.getMotorVoltage().getValueAsDouble());
    }
}