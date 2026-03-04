package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
    private final ShooterSubsystem kraken;
    private boolean isRunning = false;

    public ShooterCommand(ShooterSubsystem kraken) {
        this.kraken = kraken;
        addRequirements(kraken);
    }

    @Override
    public void initialize() {
        isRunning = !isRunning; // Toggle state on each press

        if (isRunning) {
            kraken.spin();
        } else {
            kraken.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return true; // Ends immediately, motor keeps spinning until next press
    }
}