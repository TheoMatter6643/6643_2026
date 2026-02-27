package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends Command {
    public DriveSubsystem drive;
    public Supplier<ChassisSpeeds> v;
    public DriveCommand(DriveSubsystem drive, Supplier<ChassisSpeeds> v) {
        this.drive = drive;
        this.v = v;
        this.addRequirements(drive);
    }

    @Override
    public void execute() {
        // Optional<Alliance> alliance = DriverStation.getAlliance();
        ChassisSpeeds speeds = v.get();
        // if (alliance.get() == Alliance.Red) {
        //     speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        // }
        this.drive.getSwerveDrive().driveFieldOriented(speeds);
    }
}
