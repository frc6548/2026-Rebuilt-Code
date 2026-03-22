package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX Shootermotor = new TalonFX(14);
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Ramp rate — time in seconds to go from 0 to full speed
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1.0; // 1 second to full speed — increase to ramp slower

        Shootermotor.getConfigurator().apply(config);
    }

    public void spin() {
        Shootermotor.setControl(dutyCycle.withOutput(.75));
    }

    public void stop() {
        Shootermotor.setControl(dutyCycle.withOutput(0));
    }
    
    public double getVelocityRPS() {
        return Shootermotor.getVelocity().getValueAsDouble();
    }
 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velocity (RPS)", getVelocityRPS());
        SmartDashboard.putNumber("Shooter Velocity (RPM)", getVelocityRPS() * 60.0);
         SmartDashboard.putNumber("Shooter Motor Output", Shootermotor.getMotorVoltage().getValueAsDouble());
    }
}