// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimAtAprilTagCommand;
import frc.robot.commands.FollowAprilTagCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Creates the Xbox controller to drive the robot
  CommandXboxController mainController = new CommandXboxController(0);  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /** Use this method to define your trigger->command mappings. */
  private void configureBindings() {
    // Put any trigger->command mappings here.
    
    // Run motor with setSpeeds command
    Command arcadeDrive =
      m_driveSubsystem.run(
        () -> {
          m_driveSubsystem.setArcadeSpeed(
          deadBand(-mainController.getLeftY(), 0.1),
          deadBand(mainController.getRightX(), 0.1)
          );
        }
      );
    
    Command tankDrive =
      m_driveSubsystem.run(
        () -> {
          m_driveSubsystem.setSpeeds(
          deadBand(-mainController.getLeftY(), 0.1),
          deadBand(-mainController.getRightY(), 0.1)
          );
        }
      );
    
    Command quickTurn =  
      m_driveSubsystem.run(
        () -> {
          m_driveSubsystem.setSpeeds(
          0.5,
          -0.5
          );
        }
      ).withTimeout(1.5);

    Command aimAtTag = new AimAtAprilTagCommand(m_driveSubsystem);
    Command followTag = new FollowAprilTagCommand(m_driveSubsystem);
    
    mainController.leftBumper().whileTrue(aimAtTag);
    mainController.rightBumper().whileTrue(followTag);

    m_driveSubsystem.setDefaultCommand(arcadeDrive);
    
    mainController.a().toggleOnTrue(tankDrive);

  }

  // Deadband command to eliminate drifting
  public static double deadBand(double value, double tolerance) {
    if(value < tolerance && value > -tolerance) {
      return 0;
    } else {
      return value;
    }
  }
  
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {};
  }
}
