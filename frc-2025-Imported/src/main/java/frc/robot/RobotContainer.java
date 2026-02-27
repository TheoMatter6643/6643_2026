package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.subsystems.DriveSubsystem;
import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShootCommand;

import java.util.HashSet;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
      
  private final CommandXboxController controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController buttons = new CommandXboxController(1);
  public final DriveSubsystem drive = new DriveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  SwerveInputStream driveAngularVelocity = SwerveInputStream
    // switch to negative for Blue, positive for Red
    .of(drive.getSwerveDrive(), () -> controller.getLeftY() * -1, () -> controller.getLeftX() * -1)
    // switch to positive for Blue, negitage for Red
    .withControllerRotationAxis(() -> controller.getRightX() * -1)
    .deadband(0.1)
    // .scaleTranslation(0.8)
    .allianceRelativeControl(true);
   // SwerveInputStream driveAngularVelocity;
  private final SendableChooser<Command> autoChooser;

  /* if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      SwerveInputStream driveAngularVelocity = SwerveInputStream
    .of(drive.getSwerveDrive(), () -> controller.getLeftY() * 1, () -> controller.getLeftX() * 1)
    .withControllerRotationAxis(() -> controller.getRightX() * 1)
    .deadband(0.1)
    // .scaleTranslation(0.8)
    .allianceRelativeControl(true);
  }
  else {
      SwerveInputStream driveAngularVelocity = SwerveInputStream
    .of(drive.getSwerveDrive(), () -> controller.getLeftY() * -1, () -> controller.getLeftX() * -1)
    .withControllerRotationAxis(() -> controller.getRightX() * -1)
    .deadband(0.1)
    // .scaleTranslation(0.8)
    .allianceRelativeControl(true);
  }
*/
  
  public RobotContainer() {
    configureBindings();
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // SmartDashboard.putData("Reset Odometry", autoFactory.resetOdometry("reset"));
  }

  private void configureBindings() {
    drive.setDefaultCommand(new DriveCommand(drive, driveAngularVelocity));
    //this.controller.rightTrigger().whileTrue(new LaunchSequence(ShooterSubsystem));
    //ShooterSubsystem.setDefaultCommand(Shooter.run(() -> ShooterSubsystem.stop()));
    // this.controller.rightBumper().whileTrue(new ClimbCommand(climber, 0.5));
    // this.controller.rightBumper().whileTrue(new MoveArmSetpointCommand(arm, 140));
    // this.controller.leftBumper().whileTrue(new ClimbCommand(climber, -0.5));  

    this.controller.leftBumper().onTrue(new ResetGyroCommand(drive));
    Set<Subsystem> set = new HashSet<>();
    // set.add(drive);
    this.buttons.rightBumper().whileTrue(new DeferredCommand(drive::generatePathToReef, set));
   // this.buttons.rightTrigger().whileTrue(ShooterSubsystem.());
    this.buttons.rightTrigger().whileTrue(new shoot(ShooterSubsystem));
    //115 140
   // this.buttons.rightTrigger().whileTrue(new DeferredCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
