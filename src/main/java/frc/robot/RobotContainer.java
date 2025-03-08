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
import frc.robot.subsystems.Intake;
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
  private final Intake m_intake = new Intake();
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
      // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
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


        //                                                                                 CONTROLLER CONTROLS
        new DefaultDrive(
           m_driveTrain,
           ()->Math.signum(m_driverController.getLeftY()) * Math.pow(m_driverController.getLeftY(),2),
           ()->Math.signum(m_driverController.getLeftX()) * Math.pow(m_driverController.getLeftX(),2),
           ()->Math.signum(m_driverController.getRightX()) * Math.pow(m_driverController.getRightX(),2)));
            //() -> -m_driverController.getLeftY(),
            //() -> -m_driverController.getLeftX(),
            //() -> -m_driverController.getRightX()));
// m_lift.setDefaultCommand(new Liftnudge(m_lift, ()-> MathUtil.applyDeadband( m_copilotController.getLeftY(),0.2)));
m_copilotController.povUp().whileTrue(new Liftnudge(m_lift, ()->0.2 ));
m_copilotController.povDown().whileTrue(new Liftnudge(m_lift, ()->-0.2 ));
            m_copilotController.y().onTrue(m_arm.ArmtopositionCommand(Arm.Spitarm));
            m_copilotController.b().onTrue(m_arm.ArmtopositionCommand(Arm.Middlearm));
            m_copilotController.a().onTrue(m_arm.ArmtopositionCommand(Arm.Floorarm));
           m_copilotController.x().onTrue(m_arm.ArmtopositionCommand(Arm.Reefarm));
            m_arm.setDefaultCommand(new Armnudge(m_arm, ()-> MathUtil.applyDeadband( m_copilotController.getRightY(),0.2)));
m_cage.setDefaultCommand(new RunCommand(()->m_cage.Run(0), m_cage));
//m_copilotController.povUp().whileTrue(new RunCommand(()->m_cage.Run(0.2), m_cage));
//m_copilotController.povDown().whileTrue(new RunCommand(()->m_cage.Run(-0.2), m_cage));
// m_copilotController.rightTrigger().onTrue(new RunCommand(()-> m_intake.Run(0.4),m_intake)).onFalse(new RunCommand(()->m_intake.Run(0), m_intake));
// m_copilotController.leftTrigger().onTrue(new RunCommand(()-> m_intake.Run(-0.4),m_intake)).onFalse(new RunCommand(()->m_intake.Run(0), m_intake));

m_copilotController.rightBumper().whileTrue(new RunCommand(() -> m_arm.Run(45), m_arm)).onFalse(new RunCommand(()-> m_arm.Run(0), m_arm));
m_copilotController.leftBumper().whileTrue(new RunCommand(() -> m_arm.Run(-45), m_arm)).onFalse(new RunCommand(()-> m_arm.Run(0), m_arm));

m_intake.setDefaultCommand(new RunCommand(()->{
  double min = 0.001;
  double right = m_copilotController.getRightTriggerAxis();
  double left = m_copilotController.getLeftTriggerAxis();
  if (right > min){
    m_intake.Run(right);
  }
  else if (left > min){
    m_intake.Run(-left);
  }
  else{
    m_intake.Run(0.0);
  }
}, m_intake));
//                                                                                                   BUTTON BOX CONTROLS
m_copilotButtonbox.button(4).onTrue(m_lift.LifttopositionCommand(Lift.positionL1));
m_copilotButtonbox.button(2).onTrue(m_lift.LifttopositionCommand(Lift.positionL2));
m_copilotButtonbox.button(1).onTrue(m_lift.LifttopositionCommand(Lift.positionL3));
m_copilotButtonbox.button(3).onTrue(m_lift.LifttopositionCommand(Lift.positionL4));
m_copilotButtonbox.button(5).onTrue(m_lift.LifttopositionCommand(Lift.positionNet));
m_copilotButtonbox.button(6).onTrue(m_lift.LifttopositionCommand(Lift.positionFloor));
m_copilotButtonbox.button(8).onTrue(new RunCommand(()->m_cage.Run(-0.2), m_cage)).onFalse (new RunCommand(()->m_cage.Run(0), m_cage));
m_copilotButtonbox.button(7).onTrue(new RunCommand(()->m_cage.Run(0.2), m_cage)).onFalse (new RunCommand(()->m_cage.Run(0), m_cage));
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
