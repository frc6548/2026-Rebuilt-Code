package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX Intakemotor = new TalonFX(15);
        private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    public void spin() {
        Intakemotor.setControl(dutyCycle.withOutput(.5)); // 0.5 = 50% speed, adjust as needed
    }

    public void stop() {
        Intakemotor.setControl(dutyCycle.withOutput(0));
    }


} 
