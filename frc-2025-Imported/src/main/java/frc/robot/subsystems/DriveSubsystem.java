package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import static swervelib.math.SwerveMath.calculateMaxAngularVelocity;

public class DriveSubsystem extends SubsystemBase {
    double maximum = Units.feetToMeters(3);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    SwerveDrive swerve;
    StructPublisher<Pose2d> publisher;
    RobotConfig config;
    Timer visionAlignTimer = new Timer();
    // boolean firstVisionUpdate = true;
    DoubleLogEntry poseX;
    DoubleLogEntry poseY;
    DoubleLogEntry poseRot;

    public DriveSubsystem() {
        try {
            this.swerve = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximum);
        } catch (IOException e) {
            e.printStackTrace();
        }
        publisher = NetworkTableInstance.getDefault().getStructTopic("pose", Pose2d.struct).publish();

        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }
        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getRobotRelativeSpeeds,
            (speeds, ff) -> this.swerve.drive(speeds), 
            // 2.8 3.1
            new PPHolonomicDriveController(
                new PIDConstants(5.1,0,0), 
                new PIDConstants(5.1,0,0)
            ), 
            config, 
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
            this
        );
        DataLog log = DataLogManager.getLog();
        poseX = new DoubleLogEntry(log, "/poseX");
        poseY = new DoubleLogEntry(log, "/poseY");
        poseRot = new DoubleLogEntry(log, "/poseRot");
    }
    // SPEED how to control the maximum capable speed and set in baby mode 
    public void setTeleop() {
        double vel = Units.feetToMeters(5);
        double angVel = calculateMaxAngularVelocity(
            vel,
            Units.inchesToMeters(17),
            Units.inchesToMeters(17));
        swerve.setMaximumAllowableSpeeds(vel, angVel);
        swerve.setMaximumAttainableSpeeds(vel, Math.PI * 4);
    }
    public void updateVisionOdometry() {
        if (!this.visionAlignTimer.isRunning()) {
            this.visionAlignTimer.start();
        }
        boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        if (red) {
            int[] redIds = { 6,7,8,9,10,11 };
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight", redIds);
            // LimelightHelpers.setCameraPose_RobotSpace("", 0.14, 0.216, 0.34, 0, 0, 180);
        } else {
            int[] blueIds = { 17,18,19,20,21,22 };
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight", blueIds);
            // LimelightHelpers.setCameraPose_RobotSpace("", 0.14, 0.216, 0.34, 0, 0, 0);
        }
        Pose2d pose = this.getPose(); 
        Rotation2d rot;
        // if(red && pose.getX() == 0 && pose.getY() == 0)
        // {
            // rot = pose.getRotation().unaryMinus();
        // } else {
            rot = pose.getRotation();
        // }
        // this.firstVisionUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", rot.getDegrees(), 0, 0,0,0,0);
        
        if (this.visionAlignTimer.hasElapsed(0.2)) {
            // TO DO: tune this vision align cooldown
            LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            // LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            if (limelightMeasurement != null && limelightMeasurement.tagCount >= 1) {
                Pose2d pose2 = limelightMeasurement.pose;
                // if (red) {
                //     pose2 = new Pose2d(pose2.getTranslation(), Rotation2d.fromDegrees(pose2.getRotation().getDegrees()));
                // }// fix yaw
                swerve.addVisionMeasurement(pose2, limelightMeasurement.timestampSeconds, VecBuilder.fill(.7,.7,9999999));
            }
            this.visionAlignTimer.reset();
        }
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.swerve.getRobotVelocity();
    }
    public SwerveDrive getSwerveDrive() {
        return this.swerve;
    }
    public void drive(Translation2d translation, double rotation, boolean fieldOriented, boolean slowMode) {
        this.swerve.drive(translation, rotation, fieldOriented, slowMode);
    }
    public void resetOdometry(Pose2d pose) {
        swerve.resetOdometry(pose);
    }
    public Pose2d getPose() {
        return swerve.getPose();
    }
    @Override
    public void periodic() {
        Pose2d pose = getPose();
        publisher.set(pose); 
        // poseX.append(pose.getX());
        // poseY.append(pose.getY());
        // poseRot.append(pose.getRotation().getDegrees());
    }

    public Command generatePathToReef() {
        // blue left, clockwise
        Pose2d[] reefLocations = {
            new Pose2d(4.994, 5.258, Rotation2d.fromDegrees(-120)),
            new Pose2d(5.394, 4.979, Rotation2d.fromDegrees(-120)),
            new Pose2d(5.753, 4.248, Rotation2d.fromDegrees(180)),
            new Pose2d(5.763, 3.836, Rotation2d.fromDegrees(180)),
            new Pose2d(5.160, 3.064, Rotation2d.fromDegrees(120)),
            new Pose2d(4.93, 2.833, Rotation2d.fromDegrees(120)),
        };
        // calculate closest one
        Pose2d target = null;
        double closestDistance = Double.MAX_VALUE;
        for (Pose2d reef : reefLocations) {
            Pose2d reefAdjusted = reef;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) reefAdjusted = FlippingUtil.flipFieldPose(reef);
            double distance = this.getPose().getTranslation().getDistance(reefAdjusted.getTranslation());
            
            if (distance < closestDistance) {
                closestDistance = distance;
                target = reefAdjusted;
            }
        }
        // generate path to closest reef
        PathConstraints constraints = new PathConstraints(2.0, 2.0, 2*Math.PI, 4*Math.PI);
        Command path = AutoBuilder.pathfindToPose(target, constraints, 0.0);
        return path;
    }
}
