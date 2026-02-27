package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.SwerveModule;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer; 

  public Robot() {
    robotContainer = new RobotContainer();
    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // 1: 0.87, 0.62, 0.65
    SwerveModule[] modules = robotContainer.drive.getSwerveDrive().getModules();
    SmartDashboard.putNumber("Encoder1", modules[0].getAbsolutePosition());
    SmartDashboard.putString("SM1", modules[0].configuration.name);
    SmartDashboard.putNumber("Encoder2", modules[1].getAbsolutePosition());
    SmartDashboard.putString("SM2", modules[1].configuration.name);
    SmartDashboard.putNumber("Encoder3", modules[2].getAbsolutePosition());
    SmartDashboard.putString("SM3", modules[2].configuration.name);
    SmartDashboard.putNumber("Encoder4", modules[3].getAbsolutePosition());
    SmartDashboard.putString("SM4", modules[3].configuration.name);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    this.robotContainer.drive.updateVisionOdometry();
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.drive.setTeleop();
  }

  @Override
  public void teleopPeriodic() {
    this.robotContainer.drive.updateVisionOdometry();
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
