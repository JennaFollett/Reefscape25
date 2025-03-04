// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Armnudge;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Liftnudge;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cageclimb;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.TestDriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrainBase m_driveTrain = new DriveTrain();
  private final Lift m_lift = new Lift();
  private final Arm m_arm = new Arm();
  private final Cageclimb m_cage = new Cageclimb();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      private final CommandXboxController m_copilotController =
      new CommandXboxController(OperatorConstants.kCopilotControllerPort);

      private final CommandJoystick m_copilotButtonbox = new CommandJoystick(2);
      
//private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //   autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    //SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driveTrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DefaultDrive(
           m_driveTrain,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));
m_lift.setDefaultCommand(new Liftnudge(m_lift, ()-> MathUtil.applyDeadband( m_copilotController.getLeftY(),0.2)));
            m_copilotController.y().onTrue(m_lift.LifttopositionCommand(Lift.positionL2));
            m_copilotController.b().onTrue(m_lift.LifttopositionCommand(Lift.positionL3));
            m_copilotController.a().onTrue(m_lift.LifttopositionCommand(Lift.positionL4));
            m_copilotController.x().onTrue(m_lift.LifttopositionCommand(Lift.positionNet));
            m_arm.setDefaultCommand(new Armnudge(m_arm, ()-> MathUtil.applyDeadband( m_copilotController.getRightY(),0.2)));
m_cage.setDefaultCommand(new RunCommand(()->m_cage.Run(0), m_cage));
m_copilotController.povUp().whileTrue(new RunCommand(()->m_cage.Run(0.2), m_cage));
m_copilotController.povDown().whileTrue(new RunCommand(()->m_cage.Run(-0.2), m_cage));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return autoChooser.getSelected();
    return null;
  }

}
