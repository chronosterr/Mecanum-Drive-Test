// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  // private static final int kFrontLeftChannel = 2;
  // private static final int kRearLeftChannel = 3;
  // private static final int kFrontRightChannel = 1;
  // private static final int kRearRightChannel = 0;
  WPI_TalonSRX _talonDriveFL = new WPI_TalonSRX(1);
  WPI_TalonSRX _talonDriveRL = new WPI_TalonSRX(3);
  WPI_TalonSRX _talonDriveFR = new WPI_TalonSRX(11); 
  WPI_TalonSRX _talonDriveRR = new WPI_TalonSRX(2);
  WPI_TalonSRX _talonIntake = new WPI_TalonSRX(6);
  WPI_TalonSRX _talonKicker = new WPI_TalonSRX(5);
  WPI_TalonSRX _talonShooter = new WPI_TalonSRX(4);
  WPI_TalonSRX _talonClimbWinchF = new WPI_TalonSRX(9);
  WPI_TalonSRX _talonClimbWinchR = new WPI_TalonSRX(10);
  WPI_TalonSRX _talonClimbRotateF = new WPI_TalonSRX(7);
  WPI_TalonSRX _talonClimbRotateR = new WPI_TalonSRX(8); 


  WPI_Pigeon2 gyro = new WPI_Pigeon2(12);

  double x; double y; double area; float Kp;
  float min_command; double left_command; double right_command;
  double forward_command; double backward_command; double xHeading_error;
  double yHeading_error; double x_adjust; double y_adjust;

  private static final int kLJoystickChannel = 0;
  private static final int kRJoystickChannel = 1;
  //private final PS4Controller ps4 = new PS4Controller(0);
  private final XboxController xbox = new XboxController(2); //Optional, use when coding buttons

  private MecanumDrive m_robotDrive;
  private Joystick l_stick;
  private Joystick r_stick;
  private final Timer m_timer = new Timer();

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  
  @Override
  public void robotInit() {
    // TalonSRX frontLeft = new TalonSRX(_talonFL);
    // TalonSRX rearLeft = new TalonSRX(_talonRL);
    // TalonSRX frontRight = new TalonSRX(kFrontRightChannel);
    // TalonSRX rearRight = new TalonSRX(kRearRightChannel);
    
    CameraServer.startAutomaticCapture();

    _talonDriveFL.configFactoryDefault();
    _talonDriveRL.configFactoryDefault();
    _talonDriveFR.configFactoryDefault();
    _talonDriveRR.configFactoryDefault();
    _talonIntake.configFactoryDefault();
    _talonKicker.configFactoryDefault();
    _talonShooter.configFactoryDefault();
    _talonClimbWinchF.configFactoryDefault();
    _talonClimbWinchR.configFactoryDefault();
    _talonClimbRotateF.configFactoryDefault();
    _talonClimbRotateR.configFactoryDefault();
    
    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    _talonDriveFL.setInverted(false);
    _talonDriveRL.setInverted(false);
    _talonDriveFR.setInverted(true);
    _talonDriveRR.setInverted(true);
  
   
    m_robotDrive = new MecanumDrive(_talonDriveFL, _talonDriveRL, _talonDriveFR, _talonDriveRR);
    //m_robotDrive.setRightSideInverted(false);
    r_stick = new Joystick(kRJoystickChannel);
    l_stick = new Joystick(kLJoystickChannel);

    
  }
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double heading = gyro.getAngle();
    if (m_timer.get() > 0.0 && m_timer.get() < 2.0) { //DRIVE FORWARD, 2 SECONDS
      m_robotDrive.driveCartesian(.5, 0, 0);
    }
    else if (m_timer.get() > 2.0 && m_timer.get() < 10.0) { //BETWEEN 2 AND 10 SECONDS, MOVE TO FACE 90 DEGREES
        if (heading < 89.0) {
          m_robotDrive.driveCartesian(0, 0, .4);
        }
        else if (heading > 91.0) {
          m_robotDrive.driveCartesian(0, 0, -.4);
        }
    }
  }

  public void limelightTarget() {
    xHeading_error = -x;
    yHeading_error = -y;
    x_adjust = 0.0f;
    y_adjust = 0.0f;
    if (x > 1.0)
    {
            x_adjust = Kp*xHeading_error - min_command;
    }
    else if (x < 1.0)
    {
            x_adjust = Kp*xHeading_error + min_command;
    }
    left_command += x_adjust;
    right_command -= x_adjust;

    if (y > 1.0)
    {
      y_adjust = Kp*yHeading_error - min_command;
    }
    else if (y < 1.0)
    {
      y_adjust = Kp*yHeading_error + min_command;
    }
    forward_command += y_adjust;
    backward_command -= y_adjust;

    m_robotDrive.driveCartesian(y_adjust, x_adjust, 0.0);

    /*
    In terms of rotation, if the tape were to be viewed from an angle 
    that is not facing the hub, The area of the tape (grouped together)
    would be larger. The first step to implementing this would be to
    first change the limelight settings to group the reflective tape
    marks on the Hub together (check main Limelight website FAQ for
    how to do that), and find what the area of the tape is when at
    an ideal angle. The only thing would be figuring out which way to
    rotate, which makes me think the whole thing may be a waste of time.
    I don't think it'll be a problem? Not sure. Grasping at straws here.

    */

    //https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
    /*
    Work with Wyatt to find a suitable fixed angle for the robot, find the height
    from the floor, and plug in the variables accordingly. This system will
    be much better than calculating based on area.
    */
  }
  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    //XBOX DRIVE IS BELOW
    //m_robotDrive.driveCartesian(-xbox.getLeftY(), -xbox.getLeftX(), xbox.getRightX(), 0.0);
    m_robotDrive.driveCartesian(-l_stick.getY(), -l_stick.getX(), r_stick.getZ(), 0.0);
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    Kp = -0.05f;
    min_command = 0.1f;
    left_command = 0;
    right_command = 0;
    forward_command = 0;
    backward_command = 0;

    Double[] motorValues = {_talonDriveFL.get(), _talonDriveFR.get(), _talonDriveRL.get(), _talonDriveRR.get()};

     SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
    SmartDashboard.putNumberArray("RobotDrive Motors", motorValues);

    if (r_stick.getRawButton(2) || xbox.getRawButton(8)) { //FLYWHEEL OR SHOOTER CONTROL
      _talonShooter.set(-.9);
    } else {
      _talonShooter.set(0);
    }
    
    if (r_stick.getRawButton(1) || xbox.getRawButton(6)) { //KICKER CONTROL
      _talonKicker.set(-.5);
    } else {
      _talonKicker.set(0);
    }

    if (l_stick.getRawButton(1) || xbox.getRawButton(7)) { //INTAKE CONTROL
      _talonIntake.set(.30);
    } else {
      _talonIntake.set(0);
    }

    if (r_stick.getRawButton(4) || xbox.getRawButton(4)) { //FRONT CLIMB ARM WINCH MANUAL CONTROL
      _talonClimbWinchF.set(.5);
    } else if (xbox.getRawButton(1)) {
      _talonClimbWinchF.set(-.5);
    } else {
      _talonClimbWinchF.set(0);
    
    if (r_stick.getRawButton(6) || xbox.getRawButton(2)) { //BACK CLIMB ARM ROTATION MANUAL CONTROL
      _talonClimbRotateF.set(.5);
    } else if (xbox.getRawButton(3)) {
      _talonClimbRotateF.set(-.5);
    } else {
      _talonClimbRotateF.set(0);
    }

    if (r_stick.getRawButton(3) || xbox.getPOV() == 0) { //FRONT CLIMB ARM WINCH MANUAL CONTROL
      _talonClimbWinchR.set(.5);
    } else if (xbox.getPOV() == 180) {
      _talonClimbWinchR.set(-.5);
    } else {
      _talonClimbWinchR.set(0);
    
    if (r_stick.getRawButton(6) || xbox.getPOV() == 270) { //BACK CLIMB ARM ROTATION MANUAL CONTROL
      _talonClimbRotateR.set(.5);
    } else if (xbox.getPOV() == 90) {
      _talonClimbRotateR.set(-.5);
    } else {
      _talonClimbRotateR.set(0);
    }

    if (l_stick.getRawButton(2)) { //LIMELIGHT TARGETTING (IN BETA)
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //turns on LEDs
      // limelightTarget(); //separate targetting function
      xHeading_error = -x;
      yHeading_error = -y;
      x_adjust = 0.0f;
      y_adjust = 0.0f;
      if (x > 1.0)
      {
              x_adjust = Kp*xHeading_error - min_command;
      }
      else if (x < 1.0)
      {
              x_adjust = Kp*xHeading_error + min_command;
      }
      left_command += x_adjust;
      right_command -= x_adjust;

      if (y > 1.0)
      {
        y_adjust = Kp*yHeading_error - min_command;
      }
      else if (y < 1.0)
      {
        y_adjust = Kp*yHeading_error + min_command;
      }
      forward_command += y_adjust;
      backward_command -= y_adjust;
      SmartDashboard.putNumber("Forward Command", forward_command);
      SmartDashboard.putNumber("Backward Command", backward_command);
      SmartDashboard.putNumber("Left Command", left_command);
      SmartDashboard.putNumber("Right Command", right_command);

      m_robotDrive.driveCartesian(backward_command+forward_command, left_command+right_command, 0.0);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //turns off LEDs
    }



}
}
}
}
