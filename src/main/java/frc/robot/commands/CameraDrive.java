// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.photonvision.PhotonCamera;

public class CameraDrive extends CommandBase {
  private static final double deltaTime = 0.02;

  private final Drivetrain m_drivetrain;
  private final PhotonCamera m_camera;
  private final PIDController m_controller;

  private double targetAngle;
  private double rotateSpeed;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to
   * the speed supplier lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public CameraDrive(Drivetrain drivetrain, PhotonCamera camera, double speed) {
    m_controller = new PIDController(speed, 0.02, 0);
    m_drivetrain = drivetrain;
    m_camera = camera;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_camera.setPipelineIndex(0);
    targetAngle = 90;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = m_camera.getLatestResult();

    var rotateSpeed = 0.0;
    var forwardSpeed = 0.0;

    if (!result.hasTargets()) {
      System.out.println("No target...");
      rotateSpeed = 0.25;
    } else {
      var target = result.getBestTarget();
      double yaw = target.getYaw();

      yaw = map(yaw, -25, 25, -1, 1) * 8;

      double pid_result = m_controller.calculate(yaw, 0) * 0.15;
      rotateSpeed = clamp(pid_result, -0.5, 0.5);

      System.out.println("Target found... Rotate Speed: " + rotateSpeed);
    }

    m_drivetrain.arcadeDrive(forwardSpeed, rotateSpeed);
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

  public double map(double value, double a, double b, double c, double d) {
    return (value - a) * (d - c) / (b - a) + c;
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
