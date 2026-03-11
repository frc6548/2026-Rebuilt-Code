package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase{
    private final TalonFX Spindexermotor = new TalonFX(10);
        private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    public void spin() {
        Spindexermotor.setControl(dutyCycle.withOutput(1)); // 0.5 = 50% speed, adjust as needed
    }

    public void stop() {
        Spindexermotor.setControl(dutyCycle.withOutput(0));
    }


}