// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.JoystickDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // private PhotonCamera m_camera = new PhotonCamera("photonvision");
  private PhotonCamera m_camera = new PhotonCamera("mycamera");
  
  // Assumes a gamepad plugged into channnel 0
  private final Joystick m_controller = new Joystick(0);

  private final double accel = 0.01;
  private double speed = 0;

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the
  // hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry
  // pi.
  // By default, the following are available (listed in order from inside of the
  // board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    var inst = NetworkTableInstance.create();
    inst.startDSClient();
    inst.startClient("photonvision.local", 1735);

    m_camera = new PhotonCamera(inst.getTable("photonvision/mycamera"));

    System.out.println("Subtables: " + inst.getTable("photonvision").getSubTables());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(new JoystickDrive(m_controller, m_drivetrain));
    // m_drivetrain.setDefaultCommand(new CameraDrive(m_drivetrain, m_camera, 0.5));

    JoystickButton joystickButtonA = new JoystickButton(m_controller, Constants.JOYSTICK_BUTTON_A);
    joystickButtonA.whenPressed(() -> m_drivetrain.resetGyro());

    JoystickButton joystickButtonY = new JoystickButton(m_controller, Constants.JOYSTICK_BUTTON_Y);
    joystickButtonY.whenPressed(() -> m_camera.takeInputSnapshot());

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(m_drivetrain, () -> getForwardSpeed(), () -> getTurningSpeed());
  }

  private double getForwardSpeed(){
    double input = m_controller.getRawAxis(3) - m_controller.getRawAxis(2);

    double desiredSpeed = input * 1.1;
    double accelMult = m_controller.getRawButton(1) ? 4 : 1;

    if (Math.abs(desiredSpeed) < 0.1){
      desiredSpeed = 0;
      accelMult *= 10;
    }

    speed = lerp(speed, desiredSpeed, accel * accelMult);

    return speed;
  }

  private double getTurningSpeed() {
    double input = m_controller.getRawAxis(0) * 0.6;

    return input + input * speed * 0.3;
  }

  private double getRobotAngle() {
    double a = m_drivetrain.getGyroAngleZ() % 360;

    if (a < 0) {
      a += 360;
    }

    return a;
  }

  private double getAngleBetween(double a1, double a2) {
    double diff = (a2 - a1 + 180) % 360 - 180;
    return diff < -180 ? diff + 360 : diff;
  }

  private double clamp(double value, double min, double max) {
    if (value < min) {
      return min;
    }
    if (value > max) {
      return max;
    }
    return value;
  }

  private double lerp(double from, double to, double time) {
    return from + (to - from) * clamp(time, 0, 1);
  }
}
