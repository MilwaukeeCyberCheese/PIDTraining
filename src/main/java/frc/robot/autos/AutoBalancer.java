// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Have the robot drive arcade style. */
public class AutoBalancer extends CommandBase {
  private final DriveSubsystem m_DriveSubsystem;
  private double throttle;
  private PIDController balancePid = new PIDController(Constants.Balancing.kP, Constants.Balancing.kI,
      Constants.Balancing.kD);

  /**
   * Creates a new AutoBalance Command.
   *
   * @param drivetrain The drivetrain subsystem to drive
   * @param shifter    Shifter for the gearboxes
   *
   */
  public AutoBalancer(DriveSubsystem drivetrain) {
    m_DriveSubsystem = drivetrain;
    addRequirements(m_DriveSubsystem);

  }

  @Override
  public void initialize() {

    // initialize PID
    balancePid.setSetpoint(0);
    balancePid.setTolerance(Constants.Balancing.kBalancedThresholdDegrees);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    double pitchAngleDegrees = MathUtil.clamp(Constants.Sensors.gyro.getRoll(), -Constants.Balancing.kBounceThreshold,
        Constants.Balancing.kBounceThreshold);


    // calculate and set throttle using PID
    throttle = balancePid.calculate(pitchAngleDegrees);
    m_DriveSubsystem.drive(throttle * Constants.Balancing.kBalanceSpeedMod, 0, 0, false, true, false);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    // Y-button cancels command
    if (RobotContainer.m_rightJoystick.getButtonFour()) {
      return true;
    } else {
      return false;
    }
  }

}