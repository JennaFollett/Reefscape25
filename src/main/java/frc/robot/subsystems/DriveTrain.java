package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DriveConstants;;

public class DriveTrain extends DriveTrainBase {
    private static final double kMaxSpeedMetersPerSecond = 4;
    private AHRS m_gyro = new AHRS(NavXComType.kUSB1);

 private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
 //private final Translation2d m_frontLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveDriveKinematics m_kinematics =
    new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveModule m_frontLeft =
      new SwerveModule(
        CANIds.kTestDriveTrainFrontLeftPower,
        CANIds.kTestDriveTrainFrontLeftTurn,
        CANIds.kTestDriveTrainFrontLeftEncoder
      );
      private final SwerveModule m_frontRight =
      new SwerveModule(
        CANIds.kTestDriveTrainFrontRightPower,
        CANIds.kTestDriveTrainFrontRightTurn,
        CANIds.kTestDriveTrainFrontRightEncoder
      );
      private final SwerveModule m_backLeft =
      new SwerveModule(
        CANIds.kTestDriveTrainBackLeftPower,
        CANIds.kTestDriveTrainBackLeftTurn,
        CANIds.kTestDriveTrainBackLeftEncoder
      );
      private final SwerveModule m_backRight =
      new SwerveModule(
        CANIds.kTestDriveTrainBackRightPower,
        CANIds.kTestDriveTrainBackRightTurn,
        CANIds.kTestDriveTrainBackRightEncoder
      );
 private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

      public DriveTrain(){
        ShuffleboardTab SwerveTab = Shuffleboard.getTab("Swerve");
        SwerveTab.add("Gyro", m_gyro);


      }
      public void periodic() {
        
        // This will get the simulated sensor readings that we set
        // in the previous article while in simulation, but will use
        // real values on the robot itself.
       updateOdometry();
        m_field.setRobotPose(m_odometry.getPoseMeters());
    
      }  
     public void updateOdometry() {
        m_odometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
            });
      }
    @Override
    public Rotation2d getRotation2d(){
     
      return Rotation2d.fromDegrees(-m_gyro.getAngle());
       // return m_gyro.getRotation2d().;
    }

    @Override
    SwerveDriveKinematics getSwerveDriveKinematics() {
       return m_kinematics;
    }
    @Override
    SwerveModule getFrontLeftModule() {
        return m_frontLeft;
    }

    @Override
    SwerveModule getFrontRightModule() {
        
        return m_frontRight;
    }

    @Override
    SwerveModule getBackLeftModule() {
        
       return m_backLeft;
    }

    @Override
    SwerveModule getBackRightModule() {
       
       return m_backRight;
    }

    @Override
    double getMaxSpeedMetersPerSecond() {
        return kMaxSpeedMetersPerSecond;
    }
    
}
