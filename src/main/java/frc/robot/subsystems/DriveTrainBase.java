package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public abstract class DriveTrainBase extends SubsystemBase {
    protected Field2d m_field = new Field2d();
public DriveTrainBase() {
  
  SmartDashboard.putData("Field", m_field);
}


    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed *= getMaxSpeedMetersPerSecond();
        ySpeed *= getMaxSpeedMetersPerSecond();
        rot *= getMaxSpeedMetersPerSecond();
    var swerveModuleStates =
        getSwerveDriveKinematics().toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, this.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                    DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, getMaxSpeedMetersPerSecond());
    getFrontLeftModule().setDesiredState(swerveModuleStates[0]);
    getFrontRightModule().setDesiredState(swerveModuleStates[1]);
    getBackLeftModule().setDesiredState(swerveModuleStates[2]);
    getBackRightModule().setDesiredState(swerveModuleStates[3]);
  }

  abstract double getMaxSpeedMetersPerSecond();
  abstract Rotation2d getRotation2d();
  abstract SwerveDriveKinematics getSwerveDriveKinematics();
  abstract ISwerveModule getFrontLeftModule();
  abstract ISwerveModule getFrontRightModule();
  abstract ISwerveModule getBackLeftModule();
  abstract ISwerveModule getBackRightModule();
}
