package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerFlowSubsystem;

public class RollerFlowCommand extends Command {
    private final RollerFlowSubsystem rollerFlow;
    private boolean isRunning = false;

    public RollerFlowCommand(RollerFlowSubsystem rollerFlow) {
        this.rollerFlow = rollerFlow;
        addRequirements(rollerFlow);
    }

    @Override
    public void initialize() {
        isRunning = !isRunning;
        if (isRunning) {
            rollerFlow.spin();
        } else {
            rollerFlow.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}