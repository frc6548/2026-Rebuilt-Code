package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RollerFlowSubsystem;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(ShooterSubsystem shooter, RollerFlowSubsystem rollerFlow) {
        addCommands(
            Commands.either(
                Commands.runOnce(() -> {
                    shooter.stop();
                    rollerFlow.stop();
                }, shooter, rollerFlow),
                new SequentialCommandGroup(
                    Commands.runOnce(shooter::spin, shooter),
                    new WaitCommand(2.5),
                    Commands.runOnce(rollerFlow::spin, rollerFlow)
                ),
                shooter::isRunning
            )
        );
    }
}