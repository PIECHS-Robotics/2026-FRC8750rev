// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Configs {
  private static final  double nominalVoltage = 12.0;

public static final class EasySwerveModule {
  public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
  public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

  static {
    // MK4i L2 conversion factors
    final double drivingFactor = 0.047286787; // meters per motor rotation
    final double turningFactor = 360.0;       // degrees per rotation (project uses degrees)
    final double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

    drivingConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)       // Drive current limit
      .openLoopRampRate(0.25)
      .closedLoopRampRate(0.25);

    drivingConfig.encoder
      .positionConversionFactor(drivingFactor)         // meters
      .velocityConversionFactor(drivingFactor / 60.0); // m/s

    drivingConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0.04, 0, 0)
      .outputRange(-1, 1)
      .feedForward.kV(drivingVelocityFeedForward);

    turningConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(20)       // Angle current limit
      .openLoopRampRate(0.25)
      .closedLoopRampRate(0.25);

    turningConfig.absoluteEncoder
      .inverted(false)
      .positionConversionFactor(turningFactor)         // degrees
      .velocityConversionFactor(turningFactor / 60.0)  // deg/sec
      .startPulseUs(3.88443797)
      .endPulseUs(1.94221899);

    turningConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(.04, 0, 0.000)
      .outputRange(-1, 1)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, turningFactor);   // 0..360 degrees
  }
}

  public static final class IntakeSubsystem {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig conveyorConfig = new SparkFlexConfig();

    static {
      // Configure basic settings of the intake motor
      intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(40);

      // Configure basic settings of the conveyor motor
      conveyorConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(0.5)
        .smartCurrentLimit(40);
    }
  }

  public static final class ShooterSubsystem {
    public static final SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    public static final SparkMaxConfig flywheelFollowerConfig = new SparkMaxConfig();
    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();

    static {
      // Configure basic setting of the flywheel motors
      flywheelConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .closedLoopRampRate(1.0)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(80);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      flywheelConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.0002)
          .outputRange(-1, 1);

      flywheelConfig.closedLoop
        .maxMotion
          // Set MAXMotion parameters for MAXMotion Velocity control
          .cruiseVelocity(5000)
          .maxAcceleration(10000)
          .allowedProfileError(1);

      // NEO kv is in rpm/V, while feedforward kV is in V/rpm, so use the reciprocal.
      flywheelConfig.closedLoop
        .feedForward.kV(nominalVoltage / Constants.NeoMotorConstants.kNeoKv);

      // Configure the follower flywheel motor to follow the main flywheel motor
      flywheelFollowerConfig.apply(flywheelConfig)
        .follow(Constants.ShooterSubsystemConstants.kFlywheelMotorCanId, true);

      // Configure basic setting of the feeder motor
      feederConfig
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .openLoopRampRate(1.0)
        .smartCurrentLimit(60);
    }
  }

}
