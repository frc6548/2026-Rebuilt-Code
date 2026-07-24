package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RollerFlowSubsystem;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(ShooterSubsystem shooter, RollerFlowSubsystem rollerFlow) {
        addCommands(
            new ParallelCommandGroup(
                new ShooterCommand(shooter),
                new WaitCommand(2.5)
            ),
            // Step 2: Run shooter and roller flow together
            new ParallelCommandGroup(
                new ShooterCommand(shooter),
                new RollerFlowCommand(rollerFlow)
            )
        );
    }
}