package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Mod0;
import frc.lib.config.*;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;
  private PIDController anglePid;

  // private final SparkMaxPIDController driveController;
  // private final SparkMaxPIDController angleController;

  // private final SimpleMotorFeedforward feedforward =
  //     new SimpleMotorFeedforward(
  //         Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;
    this.anglePid = new PIDController(
      Constants.Swerve.angleKP,
      Constants.Swerve.angleKI,
      Constants.Swerve.angleKD
    );

    /* Angle Encoder Config */
    this.angleEncoder = new CANCoder(moduleConstants.cancoderID);
    this.configAngleEncoder();

    /* Drive Motor Config */
    this.driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    this.driveEncoder = driveMotor.getEncoder();
    // this.driveController = driveMotor.getPIDController();
    this.configDriveMotor();

    /* Angle Motor Config */
    this.angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    this.integratedAngleEncoder = angleMotor.getEncoder();
    // this.angleController = angleMotor.getPIDController();
    this.configAngleMotor();

    this.lastAngle = getState().angle;
  }

  //public int countsPerRevolution = driveEncoder.getCountsPerRevolution();

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    //desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public void resetToAbsolute() {
    this.lastAngle = Rotation2d.fromDegrees(0);
  }

  public void resetIntegratedEncoders(){
    integratedAngleEncoder.setPosition(0);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig);
  }

  private void configDriveMotor() {
    this.driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    this.driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    this.driveMotor.setInverted(Constants.Swerve.driveInvert);
    this.driveMotor.setIdleMode(IdleMode.kBrake);
    this.driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    this.driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    // this.driveController.setP(Constants.Swerve.angleKP);
    // this.driveController.setI(Constants.Swerve.angleKI);
    // this.driveController.setD(Constants.Swerve.angleKD);
    // this.driveController.setFF(Constants.Swerve.angleKFF);
    this.driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.driveMotor.burnFlash();
    this.driveEncoder.setPosition(0.0);
  }

  private void configAngleMotor() {
    this.angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    this.angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    this.angleMotor.setInverted(Constants.Swerve.angleInvert);
    this.angleMotor.setIdleMode(IdleMode.kBrake);
    this.integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    // this.angleController.setP(Constants.Swerve.angleKP);
    // this.angleController.setI(Constants.Swerve.angleKI);
    // this.angleController.setD(Constants.Swerve.angleKD);
    // this.angleController.setFF(Constants.Swerve.angleKFF);
    this.angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.angleMotor.burnFlash();
    //resetIntegratedEncoders();
    // this.resetToAbsolute();
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    // if (isOpenLoop) {
    double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
    this.driveMotor.set(percentOutput);
    // } else {
    //   driveController.setReference(
    //       desiredState.speedMetersPerSecond,
    //       ControlType.kVelocity,
    //       0,
    //       feedforward.calculate(desiredState.speedMetersPerSecond));
    // }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d desiredAngle = (Math.abs(desiredState.speedMetersPerSecond) / Constants.Swerve.maxSpeed) < 0.01 ? this.lastAngle : desiredState.angle;
    Rotation2d currentAngle = this.getCanCoder();
    Double currentDegrees = currentAngle.getDegrees(); // -180 to 180
    Double desiredDegrees = desiredAngle.getDegrees();  // -180 to 180
    Double diffDegrees = (currentDegrees - desiredDegrees + 180) % 360 - 180;
    diffDegrees = diffDegrees < -180 ? diffDegrees + 360 : diffDegrees;
    
    Double value = this.anglePid.calculate(diffDegrees, 0);
    this.angleMotor.set(value);
  }

  // private Rotation2d getAngle() {
  //   return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  // }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(this.driveEncoder.getVelocity(), this.getCanCoder());
  }

  public double appliedAngleVoltage(){

    final double angleVoltage;
    return angleVoltage = angleMotor.getBusVoltage() * angleMotor.getAppliedOutput();

  }

  public double appliedDriveVoltage(){
    
    final double driveVoltage;
    return driveVoltage = driveMotor.getBusVoltage() * driveMotor.getAppliedOutput();

  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(

      (this.driveEncoder.getPosition() * (Constants.Swerve.wheelCircumference/(Constants.Swerve.driveGearRatio * 42))),
      this.getCanCoder()
    );
  }


}