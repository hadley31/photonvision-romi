// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class JoystickDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Joystick m_controller;

  private final double accel = 0.01;
  private double speed = 0;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to
   * the speed supplier lambdas. This command does not terminate.
   *
   * @param drivetrain          The drivetrain subsystem on which this command
   *                            will run
   * @param xaxisSpeedSupplier  Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public JoystickDrive(Joystick controller, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_controller = controller;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initializing joystick drive");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(getForwardSpeed(), getTurningSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getTurningSpeed() {
    double angle = getRobotAngle();
    double inputX = m_controller.getRawAxis(0);
    double inputY = m_controller.getRawAxis(1);
    double desiredAngleRad = Math.atan2(inputY, inputX);
    double desiredAngle = Math.toDegrees(desiredAngleRad) + 90;

    // Create a deadzone where we don't rotate
    if (inputX * inputX + inputY * inputY < 0.2) {
      return 0;
    }

    double smallestAngle = getAngleBetween(angle, desiredAngle);

    if (Math.abs(smallestAngle) > 172) {
      smallestAngle = Math.abs(smallestAngle);
    }

    if (Math.abs(smallestAngle) < 2.5) {
      return 0;
    }

    double logSmallestAngle = Math.log1p(Math.abs(smallestAngle));

    return clamp(logSmallestAngle * Math.signum(smallestAngle) * 0.15, -0.7, 0.7);
  }

  double Z_THRESHOLD = 45.0;

  private double getForwardSpeed() {
    double angle = getRobotAngle();
    double inputX = m_controller.getRawAxis(0);
    double inputY = m_controller.getRawAxis(1);
    double desiredAngleRad = Math.atan2(inputY, inputX);
    double desiredAngle = Math.toDegrees(desiredAngleRad) + 90;

    double smallestAngle = getAngleBetween(angle, desiredAngle);

    double trigger_input = m_controller.getRawAxis(3) - m_controller.getRawAxis(2);
    double accelMult = m_controller.getRawButton(2) ? 7 : 1;

    double desiredSpeed = 0;

    if (Math.abs(smallestAngle) < Z_THRESHOLD) {
      double input = Math.sqrt(inputX * inputX + inputY * inputY);
      desiredSpeed = input * Math.log1p(Math.abs(input) * Math.E) * trigger_input * 0.5;
    } else {
      // desiredSpeed = trigger_input;
      desiredSpeed = 0;
    }

    speed = lerp(speed, desiredSpeed, accel * accelMult);

    return speed;
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
