/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team2502.robot2019;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.kauailabs.navx.frc.AHRS;
//import com.team2502.robot2019.command.autonomous.ingredients.PrintAction;
import com.team2502.robot2019.command.autonomous.ingredients.PrintAction;
import com.team2502.robot2019.command.autonomous.ingredients.VoltageDriveAction;
import com.team2502.robot2019.subsystem.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{

    public static PigeonIMU PIGEON;
    public static TalonSRX RIGHT_SIDE;
    public static TalonSRX LEFT_SIDE;
    public static DrivetrainSubsystem DRIVE_TRAIN;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        PIGEON = new PigeonIMU(0);
        RIGHT_SIDE = new TalonSRX(1);
        LEFT_SIDE = new TalonSRX(2);
        DRIVE_TRAIN = new DrivetrainSubsystem();

        AutoSwitcher.putToSmartDashboard();
    }



    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
        DashboardData.update();
        Scheduler.getInstance().run(); // Does nothing when disabled
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit()
    {
//        PrintAction printAction = new PrintAction();
//
//        printAction.schedule();

        CommandCreator command = new CommandCreator(new VoltageDriveAction(0.2, 0.2, 3));

        Scheduler.getInstance().add(command);

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic()
    {
        // See robotPeriodic
    }

    @Override
    public void teleopInit()
    {

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
    {
        // See robotPeriodic
    }

    @Override
    public void disabledInit()
    {

    }

    @Override
    public void disabledPeriodic()
    {

    }

}
