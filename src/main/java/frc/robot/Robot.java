/*todo: 
- collector's motors in opposition + eject code
- shooter code makes collector motors run same direction (also part of limelight shooting code)
- climbing: winch motors (2) and angle ajusting motors (2) staggered, manual control from operator
- 
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

// Drive motor declarations, numbers need checked
  WPI_VictorSPX FRMotor = new WPI_VictorSPX(1);
	WPI_VictorSPX BRMotor = new WPI_VictorSPX(2);
	WPI_VictorSPX FLMotor = new WPI_VictorSPX(3);
	WPI_VictorSPX BLMotor = new WPI_VictorSPX(4);

  WPI_VictorSPX TCMotor = new WPI_VictorSPX(5);
	WPI_VictorSPX BCMotor = new WPI_VictorSPX(6);
	WPI_VictorSPX ACRMotor = new WPI_VictorSPX(7);
	WPI_VictorSPX ACLMotor = new WPI_VictorSPX(8);

  WPI_VictorSPX FCMotor = new WPI_VictorSPX(9);

  CANSparkMax SMotor= new CANSparkMax(10, MotorType.kBrushless);



  private MotorControllerGroup m_LeftMotors = new MotorControllerGroup(FLMotor,BLMotor);
  private MotorControllerGroup m_RightMotors = new MotorControllerGroup(FRMotor,BRMotor);
  private DifferentialDrive m_Drive = new DifferentialDrive(m_LeftMotors,m_RightMotors);

//Joystick declarations, numbers need checked
Joystick joyE = new Joystick(0);
Joystick joyL = new Joystick(1);
Joystick joyR = new Joystick(2);

//Joystick button variables, names subject to change
boolean joyETrigger;
boolean joyELimelightControl;
boolean joyLTrigger; //Left or right triggers control collector?
boolean joyRTrigger;
double leftValue; //Right, left, and operator joystick values
double rightValue;
double extraValue;
double manualShooterSpeed; //Controlled by flap on operator joystick, hopefully rely on limelight 98% of time

//Other variables
Timer autoTimer = new Timer();
SendableChooser Position = new SendableChooser<>();
SendableChooser Action = new SendableChooser<>();
double autoSubtractionAmount;
String selectedAction;
String selectedPosition;
double currentAutoTime = 0;
double speedLimitAmount = 1;

//Limelight code
private boolean m_LimelightHasValidTarget = false;
private double m_LimelightSteerCommand = 0.0;
double shooterPower = 0.0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    Position.addOption("Left", 1);
    Position.addOption("Mid Left", 2);
    Position.addOption("Mid Right", 3);
    Position.addOption("Right", 4);
    Action.addOption("Stationary", 1);
    Action.addOption("Move Off Tarmac", 2);
    Action.addOption("Attempt To Score", 3);
    
    SmartDashboard.putData("Action", Action);
    SmartDashboard.putData("Position", Position);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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

//Sample autonomous code, not sure how robot knows when autonomous starts/ends
  autoTimer.reset();
  autoTimer.start();
  leftValue = 0;
  rightValue = 0;
  autoSubtractionAmount = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    FRMotor.set(-rightValue); //Not sure what these do, looks important
		BRMotor.set(-rightValue);
		FLMotor.set(leftValue);
		BLMotor.set(leftValue);
		
		selectedAction = Action.getSelected().toString();
		selectedPosition = Position.getSelected().toString();
		currentAutoTime = autoTimer.get() - autoSubtractionAmount;

		SmartDashboard.putNumber("Current Auto Time", currentAutoTime);
		SmartDashboard.putNumber("RightVal", rightValue);
		SmartDashboard.putNumber("LeftVal", leftValue);

/*------------------------------Autonomous code based on what is selected------------------------------*/
    //Stationary action, any position 
    if (selectedAction.equals("1")) {
      leftValue = 0;
      rightValue = 0;
    }
    /*Simple move action, any position. We will have to face the robot in the desired direction at competition, time and speed values
    will definitely have to be changed*/
    if (selectedAction.equals("2")) {
      if (currentAutoTime > 5) {
        leftValue = 0.2;
        rightValue = 0.2;
      } else { 
        leftValue = 0;
        rightValue = 0;
      }
    }
    /*Try to shoot, position dependent to give limelight a better view, 
    rotation vales depend on how we decide to position robot, will need to test
    
    it sure isn't pretty lol*/
  if (selectedAction.equals("3")) {
      if (selectedPosition.equals("1")) {
        if (currentAutoTime > 10) {
          rightValue = 0.2;
          leftValue = 0.2;
        } else {
          //Clockwise rotation
          rightValue = -0.1;
          leftValue = 0.1;
          /*At this point the limelight will override robot turning when it sees the target, make additional distance/rotation
          adjustments and power the shooter motor when it's finished*/

          }
        }

        if (selectedPosition.equals("2")) {
          if (currentAutoTime > 10) {
            rightValue = 0.2;
            leftValue = 0.2;
          } else {
            //Clockwise rotation
            rightValue = -0.1;
            leftValue = 0.1;
            /*At this point the limelight will override robot turning when it sees the target, make additional distance/rotation
            adjustments and power the shooter motor when it's finished*/
          }
        }
      
        if (selectedPosition.equals("3")) {
          if (currentAutoTime > 10) {
            rightValue = 0.2;
            leftValue = 0.2;
          } else {
            //Counterclockwise rotation
            rightValue = 0.1;
            leftValue = -0.1;
            /*At this point the limelight will override robot turning when it sees the target, make additional distance/rotation
            adjustments and power the shooter motor when it's finished*/
          }
        }

        if (selectedPosition.equals("4")) {
          if (currentAutoTime > 10) {
            rightValue = 0.2;
            leftValue = 0.2;
          } else {
            //Counterclockwise rotation
            rightValue = 0.1;
            leftValue = -0.1;
            /*At this point the limelight will override robot turning when it sees the target, make additional distance/rotation
            adjustments and power the shooter motor when it's finished*/
          }
        }
      } 
  }
/** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    currentAutoTime = autoTimer.get();
		rightValue = joyR.getY()*speedLimitAmount;
		leftValue = joyL.getY()*speedLimitAmount;
		extraValue = joyE.getY();
    joyETrigger = joyE.getTrigger();
    boolean joyEThumbButton = joyE.getRawButton(2);
    double speedMultiplier = 0.8;

    if (joyETrigger = true) {
      SMotor.set(1*speedMultiplier);
    }

  //Limelight code
  Update_Limelight_Tracking();

  boolean auto = joyEThumbButton;

  if (auto)
  {
    if (m_LimelightHasValidTarget)
    {
          m_Drive.arcadeDrive(0.0,m_LimelightSteerCommand);
          //todo feed ball into shooter
    }
    else
    {
          m_Drive.arcadeDrive(0.0,0.0);
    }
  }
  else
  {
    FRMotor.set(rightValue);
		BRMotor.set(rightValue);
		FLMotor.set(-leftValue);
		BLMotor.set(-leftValue);
  }
}

public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
        final double maxShooterPower = 0.9; //shooter power
        final double maxDistance = 27; //feet
        final double shooterScaling = 0.1; 

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double distance = calculateDistance(); 
        double shooterPowerSubtract = (maxDistance - distance) * shooterScaling; //feet
        shooterPower = maxShooterPower - shooterPowerSubtract;


        // don't let the robot drive too fast into the goal
      /*  if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd; */
  }

  public double calculateDistance() {
    final double limelightAngle = 30; //degrees
    final double limelightHeight = 24; //inches
    final double targetHeight = 104; //inches
    double a2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    double distance = (targetHeight - limelightHeight)/Math.tan(limelightAngle + a2);

    return distance;
  }  

@Override
public void testPeriodic() {
}

/**
* This function implements a simple method of generating driving and steering commands
* based on the tracking data from a limelight camera.
*/

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
