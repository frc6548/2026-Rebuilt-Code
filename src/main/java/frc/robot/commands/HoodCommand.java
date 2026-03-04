package frc.robot.commands;

import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class HoodCommand extends Command {

    private final HoodSubsystem hoodSubsystem;
    private final double targetPosition;

    public HoodCommand(HoodSubsystem hoodSubsystem, double targetPosition) {
        this.hoodSubsystem = hoodSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println(">>> HoodCommand STARTED, target: " + targetPosition);
        System.out.println(">>> Current position at start: " + hoodSubsystem.getPosition());
        hoodSubsystem.setPosition(targetPosition);
    }

    @Override
    public void execute() { }

    @Override
    public boolean isFinished() {
        boolean done = hoodSubsystem.atTarget(targetPosition);
        System.out.println(">>> isFinished check: " + done + " | pos: " + hoodSubsystem.getPosition());
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(">>> HoodCommand ENDED, interrupted: " + interrupted);
        hoodSubsystem.stop();
    }
}