package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

/**
 * Starts the shooter, waits 1 second for it to spin up, then starts the spindexer.
 * Both run together after the delay, then stop when the command ends.
 */
public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(ShooterSubsystem shooter, SpindexerSubsystem spindexer) {
        addCommands(
            // Step 1: Start shooter, wait 1 second for spin up
            new ParallelCommandGroup(
                new ShooterCommand(shooter),
                new WaitCommand(1.0)
            ),
            // Step 2: Run shooter and spindexer together
            new ParallelCommandGroup(
                new ShooterCommand(shooter),
                new SpindexerCommand(spindexer)
            )
        );
    }
}