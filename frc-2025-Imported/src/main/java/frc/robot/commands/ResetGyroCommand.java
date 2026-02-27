package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyroCommand extends Command {
    private DriveSubsystem drive;
    public ResetGyroCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        this.drive.resetOdometry(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}