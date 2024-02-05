// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String crossTheLine = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final Timer timer = new Timer();
  //Define the motors
  private static CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private static CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private static CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private static CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);

  private static final RelativeEncoder leftEncoder = leftMotor1.getEncoder();
  private static final RelativeEncoder rightEncoder = rightMotor1.getEncoder();

  //private static CANSparkMax shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
  //private static GenericHID leftJoystick = new GenericHID(0);
  //private static GenericHID rightJoystick = new GenericHID(1); 
  private static GenericHID logitechJoystick = new GenericHID(2);

  private static final double gearReduction = 10.73;
  private static final double circumferenceMeters = Math.PI * 0.1016;

  // leftMotor1 and rightMotor1 are the leader motors
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotor1, rightMotor1);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "8424-2024-Code"); // Set a metadata value

    if (isReal()) {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", crossTheLine);
    SmartDashboard.putData("Auto choices", m_chooser);

    leftMotor1.restoreFactoryDefaults();
    leftMotor2.restoreFactoryDefaults();
    rightMotor1.restoreFactoryDefaults();
    rightMotor2.restoreFactoryDefaults();
    
    //Motor 2 will always follow Motor 1
    //Don't ever tell Motor 2 to drive!
    leftMotor1.setInverted(true);
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    leftEncoder.setPositionConversionFactor(circumferenceMeters/gearReduction);
    rightEncoder.setPositionConversionFactor(circumferenceMeters/gearReduction);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case crossTheLine:
        // Put custom auto code here
        
        if(leftEncoder.getPosition() < 1){
          diffDrive.arcadeDrive(.5, 0);
        } else{
          diffDrive.arcadeDrive(0, 0);
        }
        break;
      case kDefaultAuto:
      // Make your code here!
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double xSpeed = Math.pow(logitechJoystick.getRawAxis(1), 3);
    double zSpeed = Math.pow(logitechJoystick.getRawAxis(4), 3);
    
    diffDrive.arcadeDrive(xSpeed, zSpeed);

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
