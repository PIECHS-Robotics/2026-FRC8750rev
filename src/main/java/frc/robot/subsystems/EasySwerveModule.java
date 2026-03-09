// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import frc.robot.Configs;

public class EasySwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  // Keep this in DEGREES
  private double m_chassisAngularOffsetDeg = 0.0;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public EasySwerveModule(
      int drivingCANId,
      int turningCANId,
      double chassisAngularOffsetDeg,
      boolean drivingMotorOnBottom,
      boolean turningMotorOnBottom) {

    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    SparkMaxConfig drivingConfig = Configs.EasySwerveModule.drivingConfig;
    drivingConfig.inverted(drivingMotorOnBottom);
    m_drivingSpark.configure(
        drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig turningConfig = Configs.EasySwerveModule.turningConfig;
    turningConfig.inverted(!turningMotorOnBottom);
    m_turningSpark.configure(
        turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chassisAngularOffsetDeg = chassisAngularOffsetDeg;

    // Absolute encoder assumed configured to return DEGREES (0..360)
    m_desiredState.angle = Rotation2d.fromDegrees(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    double angleDeg = m_turningEncoder.getPosition() - m_chassisAngularOffsetDeg;
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        Rotation2d.fromDegrees(angleDeg));
  }

  public SwerveModulePosition getPosition() {
    double angleDeg = m_turningEncoder.getPosition() - m_chassisAngularOffsetDeg;
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        Rotation2d.fromDegrees(angleDeg));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset in DEGREES
    SwerveModuleState correctedDesiredState = new SwerveModuleState(
        desiredState.speedMetersPerSecond,
        desiredState.angle.plus(Rotation2d.fromDegrees(m_chassisAngularOffsetDeg)));

    // Current module angle from absolute encoder in DEGREES
    Rotation2d currentAngle = Rotation2d.fromDegrees(m_turningEncoder.getPosition());

    // Optimize to avoid >90 deg steering flips
    correctedDesiredState.optimize(currentAngle);

    // Drive velocity in m/s (assuming drive conversion factors are configured correctly)
    m_drivingClosedLoopController.setSetpoint(
        correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);

    // Turn setpoint in DEGREES (must match turning encoder position conversion factor = 360)
    m_turningClosedLoopController.setSetpoint(
        correctedDesiredState.angle.getDegrees(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}