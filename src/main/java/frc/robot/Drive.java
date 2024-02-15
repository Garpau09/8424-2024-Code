// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Drive extends SubsystemBase {

    private static CANSparkMax leftFront;
    private static CANSparkMax leftBack;
    private static CANSparkMax rightFront;
    private static CANSparkMax rightBack;

    private static RelativeEncoder leftEncoder;
    private static RelativeEncoder rightEncoder;

    private static double circumference = Math.PI * .1016;
    private static double gearRatio = 8.95;
    private static double positionConversionFactor = circumference/gearRatio;
    private static double velocityConversionFactor = (60 * circumference)/gearRatio;
    private static double trackWidth = .5334;
    private static double proportionalGain = 0;
    private static double integralGain = 0;
    private static double derivativeGain = 0;

    AHRS gyro;

    private Field2d field = new Field2d();

    DifferentialDriveOdometry odometry;

    DifferentialDriveKinematics kinematics;

    SparkPIDController leftPID;
    SparkPIDController rightPID;

    public Drive(){
        leftFront = new CANSparkMax(1, MotorType.kBrushless);
        leftBack = new CANSparkMax(2, MotorType.kBrushless);
        rightFront = new CANSparkMax(3, MotorType.kBrushless);
        rightBack = new CANSparkMax(4, MotorType.kBrushless);

        configureSparkMax(leftFront);
        configureSparkMax(leftBack);
        configureSparkMax(rightFront);
        configureSparkMax(rightBack);

        leftBack.follow(leftFront);
        rightBack.follow(rightFront);
        leftFront.setInverted(true);

        leftEncoder = leftFront.getEncoder();
        rightEncoder = rightFront.getEncoder();

        leftPID = leftFront.getPIDController();
        rightPID = rightFront.getPIDController();

        leftPID.setP(proportionalGain);
        leftPID.setI(integralGain);
        leftPID.setD(derivativeGain);

        rightPID.setP(proportionalGain);
        rightPID.setI(integralGain);
        rightPID.setD(derivativeGain);

        gyro = new AHRS(SerialPort.Port.kMXP);

        odometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(), 
            leftFront.getEncoder().getPosition(), 
            rightFront.getEncoder().getPosition());

        kinematics = new DifferentialDriveKinematics(trackWidth);

        AutoBuilder.configureRamsete(
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, 
            this::driveRobot, 
            new ReplanningConfig(), 
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                }, 
            this);        
    }

    private void configureSparkMax(CANSparkMax motor){
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.enableVoltageCompensation(12);
        motor.burnFlash();

        motor.getEncoder().setPosition(0);
        motor.getEncoder().setPosition(0.0);
        motor.getEncoder().setMeasurementPeriod(10);
        motor.getEncoder().setAverageDepth(8);
    }

    public double getGyro(){
        return gyro.getAngle();
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose){
        odometry.resetPosition(
            gyro.getRotation2d(), 
            getPositions(), 
            pose);
    }

    public DifferentialDriveWheelPositions getPositions(){
        return new DifferentialDriveWheelPositions(
            leftEncoder.getPosition() * positionConversionFactor, 
            rightEncoder.getPosition() * positionConversionFactor);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity() * velocityConversionFactor, 
            rightEncoder.getVelocity() * velocityConversionFactor);
    }

    public ChassisSpeeds getSpeeds(){
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public void driveRobot(ChassisSpeeds robotSpeeds){
        DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(robotSpeeds);
        
        leftPID.setReference(
            speeds.leftMetersPerSecond / velocityConversionFactor, 
            CANSparkBase.ControlType.kVelocity);
        rightPID.setReference(
            speeds.rightMetersPerSecond / velocityConversionFactor, 
            CANSparkBase.ControlType.kVelocity);
    }

    
}
