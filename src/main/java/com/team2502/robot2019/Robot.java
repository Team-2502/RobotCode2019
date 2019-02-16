/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team2502.robot2019;

import com.github.ezauton.core.action.ActionGroup;
import com.github.ezauton.core.action.BackgroundAction;
import com.github.ezauton.core.action.IAction;
import com.github.ezauton.core.action.PPCommand;
import com.github.ezauton.core.pathplanning.PP_PathGenerator;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.PPWaypoint;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.kauailabs.navx.frc.AHRS;
//import com.team2502.robot2019.command.autonomous.ingredients.PrintAction;
import com.team2502.robot2019.command.autonomous.ingredients.PointDriveAction;
import com.team2502.robot2019.command.autonomous.ingredients.VelocityDriveCommand;
import com.team2502.robot2019.command.autonomous.ingredients.VoltageDriveAction;
import com.team2502.robot2019.command.vision.AlwaysListeningCommand;
import com.team2502.robot2019.command.vision.GoToTargetCommand;
import com.team2502.robot2019.subsystem.CargoSubsystem;
import com.team2502.robot2019.subsystem.ClimberSubsystem;
import com.team2502.robot2019.subsystem.DrivetrainSubsystem;
import com.team2502.robot2019.subsystem.solenoid.HatchIntakeSolenoid;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{

    public static AHRS NAVX;

    public static DrivetrainSubsystem DRIVE_TRAIN;
    public static HatchIntakeSolenoid HATCH_INTAKE_SOLENOID;
    public static CargoSubsystem CARGO_ACTIVE;
    public static ClimberSubsystem CLIMBER;
    public static Compressor COMPRESSOR;
    public static UsbCamera CAMERA0;
    public static UsbCamera CAMERA1;
    public static VideoSink SERVER;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        NAVX = new AHRS(SPI.Port.kMXP);

        CAMERA0 = CameraServer.getInstance().startAutomaticCapture(0);
        CAMERA1 = CameraServer.getInstance().startAutomaticCapture(1);
        SERVER = CameraServer.getInstance().getServer();

        SERVER.setSource(CAMERA0);


        DRIVE_TRAIN = new DrivetrainSubsystem();
        HATCH_INTAKE_SOLENOID = new HatchIntakeSolenoid();
        CARGO_ACTIVE = new CargoSubsystem();
        CLIMBER = new ClimberSubsystem();
        COMPRESSOR = new Compressor();

        AutoSwitcher.putToSmartDashboard();

        NAVX.reset();
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

//        Scheduler.getInstance().add(AutoSwitcher.getAutoInstance());

        Scheduler.getInstance().add(PPTest());
        try
        {
            Scheduler.getInstance().add(new VelocityDriveCommand(.5, .5, 3));
//            ActionGroup group = new ActionGroup();
//            PointDriveAction pdAction = new PointDriveAction(10, new ImmutableVector(0, 10), 10);
//            group.with(new BackgroundAction(10, TimeUnit.MILLISECONDS, DRIVE_TRAIN::update));
//            group.addSequential((IAction) pdAction);
//            Scheduler.getInstance().add(new CommandCreator(group));
//            Scheduler.getInstance().add(new GoToTargetCommand());
//            BackgroundAction loc = new BackgroundAction(5, TimeUnit.MILLISECONDS, DRIVE_TRAIN::update);
//            ActionGroup group = new ActionGroup();
//            group.with(loc);
//            group.addSequential((IAction) new GoToTargetCommand());
//            group.addSequential((IAction) new PointDriveAction(10, new ImmutableVector(0, 10), 15));
//            Scheduler.getInstance().add(new CommandCreator(group));
        }
        catch(Exception e)
        {
            DriverStation.reportError("whoops!!!!", e.getStackTrace());
        }
//        Scheduler.getInstance().add(PPTest());

    }

    private CommandCreator PPTest() {
        PPWaypoint[] waypoints = new PPWaypoint.Builder()
                .add(0, 0, 1.6, 13, -12)
                .add(0, 4, 1.6, 13, -12)
                .add(4, 4, 1.6, 13, -12)
                .add(4, 0, 1.6, 13, -12)
                .buildArray();
        PP_PathGenerator pathGenerator = new PP_PathGenerator(waypoints);
        Path path = pathGenerator.generate(0.05);

        PurePursuitMovementStrategy ppMoveStrat = new PurePursuitMovementStrategy(path, 0.001);
        PPCommand pp = new PPCommand(10, TimeUnit.MILLISECONDS, ppMoveStrat, DRIVE_TRAIN.getLocEstimator(), Constants.Autonomous.getLookaheadBounds(DRIVE_TRAIN), DRIVE_TRAIN);
        BackgroundAction loc = new BackgroundAction(5, TimeUnit.MILLISECONDS, DRIVE_TRAIN::update);
        ActionGroup group = new ActionGroup();
        group.with(loc);
        group.addSequential(pp);
        return new CommandCreator(group);
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
