package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerFlowSubsystem extends SubsystemBase {
    private final TalonFX shooterRollers = new TalonFX(11);
    private final TalonFX singleRoller   = new TalonFX(24);
    private final TalonFX hopperRollers  = new TalonFX(25);

    private final VoltageOut voltageOut = new VoltageOut(0);

    // 11.5V compensation base
    private static final double COMP_VOLTAGE   = 11.5;
    private static final double ROLLER_SPEED   = 0.45;
    private static final double HOPPER_SPEED   = 0.15; // slow — tune if needed
    private static final double TARGET_VOLTAGE        = COMP_VOLTAGE * ROLLER_SPEED; // 5.175V
    private static final double HOPPER_TARGET_VOLTAGE = COMP_VOLTAGE * HOPPER_SPEED; // 1.725V

    public RollerFlowSubsystem() {
        // Shooter rollers config
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode    = NeutralModeValue.Coast;
        shooterConfig.Voltage.PeakForwardVoltage = COMP_VOLTAGE;
        shooterConfig.Voltage.PeakReverseVoltage = -COMP_VOLTAGE;

        // Single roller config
        TalonFXConfiguration singleRollerConfig = new TalonFXConfiguration();
        singleRollerConfig.MotorOutput.NeutralMode    = NeutralModeValue.Coast;
        singleRollerConfig.Voltage.PeakForwardVoltage = COMP_VOLTAGE;
        singleRollerConfig.Voltage.PeakReverseVoltage = -COMP_VOLTAGE;
        singleRollerConfig.CurrentLimits.StatorCurrentLimit       = 40;
        singleRollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        singleRollerConfig.CurrentLimits.SupplyCurrentLimit       = 30;
        singleRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Hopper config — slow speed, clockwise positive
        TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.MotorOutput.NeutralMode    = NeutralModeValue.Coast;
        hopperConfig.MotorOutput.Inverted       = InvertedValue.Clockwise_Positive;
        hopperConfig.Voltage.PeakForwardVoltage = COMP_VOLTAGE;
        hopperConfig.Voltage.PeakReverseVoltage = -COMP_VOLTAGE;
        hopperConfig.CurrentLimits.StatorCurrentLimit       = 80;
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hopperConfig.CurrentLimits.SupplyCurrentLimit       = 60;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterRollers.getConfigurator().apply(shooterConfig);
        singleRoller.getConfigurator().apply(singleRollerConfig);
        hopperRollers.getConfigurator().apply(hopperConfig);
    }

    public void spin() {
        shooterRollers.setControl(voltageOut.withOutput(TARGET_VOLTAGE));
        singleRoller.setControl(voltageOut.withOutput(TARGET_VOLTAGE));
        hopperRollers.setControl(voltageOut.withOutput(HOPPER_TARGET_VOLTAGE));
    }

    public void stop() {
        shooterRollers.setControl(voltageOut.withOutput(0));
        singleRoller.setControl(voltageOut.withOutput(0));
        hopperRollers.setControl(voltageOut.withOutput(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterRollers Velocity (RPS)", shooterRollers.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("SingleRoller Velocity (RPS)",   singleRoller.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("HopperRollers Velocity (RPS)",  hopperRollers.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("ShooterRollers Output",         shooterRollers.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("SingleRoller Output",           singleRoller.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("HopperRollers Output",          hopperRollers.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("SingleRoller Stator Current",   singleRoller.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("SingleRoller Supply Current",   singleRoller.getSupplyCurrent().getValueAsDouble());
    }
}