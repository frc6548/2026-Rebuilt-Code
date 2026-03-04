package frc.robot.commands;

import frc.robot.subsystems.IntakeLiftSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeLiftCommand extends Command {

    private final IntakeLiftSubsystem intakeLiftSubsystem;
    private final double targetPosition;

    public IntakeLiftCommand(IntakeLiftSubsystem intakeLiftSubsystem, double targetPosition) {
        this.intakeLiftSubsystem = intakeLiftSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(intakeLiftSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println(">>> IntakeLiftCommand STARTED, target: " + targetPosition);
        intakeLiftSubsystem.setPosition(targetPosition);
    }

    @Override
    public void execute() { }

    @Override
    public boolean isFinished() {
     boolean done = intakeLiftSubsystem.atTarget(targetPosition);
    System.out.println(">>> done: " + done + " | pos: " + intakeLiftSubsystem.getPosition() + " | target: " + targetPosition);
    return done;

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(">>> IntakeLift ENDED, interrupted: " + interrupted);
         intakeLiftSubsystem.snapshotPosition();
    }
}