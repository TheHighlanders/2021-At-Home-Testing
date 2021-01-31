// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import java.lang.reflect.Array;
import java.util.List;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);

  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTableEntry centerXEntry;
  NetworkTableEntry centerYEntry;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final Drivetrain m_drive = new Drivetrain();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;

  double[] centerXDouble;
  double[] centerYDouble;
  double[] defaultValue = new double[]{1,2,3};
  double firstX;

  @Override
  public void robotInit() {
    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every iteration.
    setNetworkTablesFlushEnabled(true);


    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, new Rotation2d()),
            List.of(
            new Translation2d(10, 6),
            new Translation2d(8, 4)
            ),
            new Pose2d(6, 4, new Rotation2d()),
            new TrajectoryConfig(2, 2));

      //Get the default instance of NetworkTables that was created automatically
       //when your program starts
       NetworkTableInstance inst = NetworkTableInstance.getDefault();

       //Get the table within that instance that contains the data. There can
       //be as many tables as you like and exist to make it easier to organize
       //your data. In this case, it's a table called datatable.
       NetworkTable table = inst.getTable("GRIP/myContoursReport");

       //Get the entries within that table that correspond to the X and Y values
       //for some operation in your program.
       xEntry = table.getEntry("X");
       yEntry = table.getEntry("Y");
       centerXEntry = table.getEntry("centerX");
       centerYEntry = table.getEntry("centerY");
       }

       double x = 0;
       double y = 0;

            

  @Override
  public void robotPeriodic() {
    m_drive.periodic();

    xEntry.setDoubleArray(centerXEntry.getDoubleArray(defaultValue));
    yEntry.setDoubleArray(centerYEntry.getDoubleArray(defaultValue));

  

  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    m_drive.resetOdometry(m_trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);




  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void teleopPeriodic() {

    centerXDouble = centerXEntry.getDoubleArray(defaultValue);
    //firstX = centerXEntry.getDoubleArray(0)[0];
    System.out.println(firstX);
    //firstX = centerXDouble[0];
    //centerXEntry.getDoubleArray(defaultValue)[0];
    firstX = Array.getDouble(centerXEntry.getDoubleArray(defaultValue), 0);

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = firstX/100;
        //-m_speedLimiter.calculate(m_controller.getY(GenericHID.Hand.kLeft)) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot =
        -m_rotLimiter.calculate(m_controller.getX(GenericHID.Hand.kLeft))
            * Drivetrain.kMaxAngularSpeed;
    m_drive.drive(xSpeed, rot);
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }
}
