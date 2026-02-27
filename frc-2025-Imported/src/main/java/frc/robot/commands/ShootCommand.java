package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
public class ShootCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private double speed;
    public ShootCommand(ShooterSubsystem shooterSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }
    @Override
    public void execute() {
        shooterSubsystem.shoot(speed);
    }
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.shoot(0);
    }
}