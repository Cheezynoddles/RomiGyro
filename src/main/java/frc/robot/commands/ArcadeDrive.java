// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double delta = 0;
    double speedAdjust = 0.95;
    double xSpeed = m_xaxisSpeedSupplier.get();

    if (m_drivetrain.isTurnLock()){
      delta = turnDelta();
      boolean invert = (m_xaxisSpeedSupplier.get() < 0);
    
      if (Math.abs(xSpeed) == 1) {
        xSpeed *= 0.9;
      }

      if (Math.abs(delta) > 15) { 
        speedAdjust = .7;
      } else if (Math.abs(delta) > 30) {
        speedAdjust = .6;
      }


      
      if ( !invert && (delta < 0) || (invert && delta > 0)) {
        m_drivetrain.tankDrive(xSpeed, xSpeed * speedAdjust);

      } else if ( !invert && (delta > 0) || (invert && delta < 0)) {

        m_drivetrain.tankDrive(xSpeed * speedAdjust, xSpeed);
      }
    } 

    if (delta == 0) {
    m_drivetrain.arcadeDrive(m_xaxisSpeedSupplier.get(), m_zaxisRotateSupplier.get());
    }
    if (delta >= Constants.GYRO_DEADBAND + 2) {
      System.out.println("Speed: " + m_xaxisSpeedSupplier.get());
      System.out.println("Adjust: " + speedAdjust);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  private double turnDelta() {
    double delta = m_drivetrain.getGyroAngleZ() - m_drivetrain.getTargetAngle();
    if(delta > 180) delta -= 360;
    if (Math.abs(delta) <= Constants.GYRO_DEADBAND) {
      return 0;
    } else {
      return delta;
    }
  }
}
