// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Commands.*;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.SmartShuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private DriveSubsystem driveTrain;
  private Joystick joystick = new Joystick(Constants.OIConstants.kJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveSubsystem();

    // Configure the button bindings
    configureButtonBindings();
    configureShuffleboard();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveTrain.setDefaultCommand(new Drive(driveTrain,
                                             () -> -joystick.getY(),
                                             () -> -joystick.getX()));
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        // If you are using the keyboard as a joystick, it is recommended that you go
        // to the following link to read about editing the keyboard settings.
        // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html#using-the-keyboard-as-a-joystick
  }

  private void configureButtonBindings() {
  }

  private void configureShuffleboard() {
  }

  public DriveSubsystem getRobotDrive() {
    return driveTrain;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroAllOutputs() {
    driveTrain.tankDriveVolts(0, 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PrintCommand("autonomous command");
  }
}
