package frc.robot.commands;

import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class HoodCommand extends Command {
    private final HoodSubsystem hood;
    private final double targetPosition;

    public HoodCommand(HoodSubsystem hood, double targetPosition) {
        this.hood = hood;
        this.targetPosition = targetPosition;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
        System.out.println(">>> HoodCommand STARTED, target: " + targetPosition);
        hood.setPosition(targetPosition);
    }

    @Override
    public void execute() { }

    @Override
    public boolean isFinished() {
        return hood.atTarget(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(">>> HoodCommand ENDED, interrupted: " + interrupted);
        hood.snapshotPosition(); // save where we stopped so holdPosition() keeps it there
    }
}