/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team2502.robot2019;

import com.github.ezauton.core.action.ActionGroup;
import com.github.ezauton.core.action.BackgroundAction;
import com.github.ezauton.core.action.PurePursuitAction;
import com.github.ezauton.core.action.tangible.MainActionScheduler;
import com.github.ezauton.core.pathplanning.PP_PathGenerator;
import com.github.ezauton.core.pathplanning.Path;
import com.github.ezauton.core.pathplanning.purepursuit.PPWaypoint;
import com.github.ezauton.core.pathplanning.purepursuit.PurePursuitMovementStrategy;
import com.github.ezauton.core.utils.MathUtils;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.wpilib.command.CommandCreator;
import com.team2502.robot2019.subsystem.CargoSubsystem;
import com.team2502.robot2019.subsystem.ClimberSubsystem;
import com.team2502.robot2019.subsystem.CrawlerSubsystem;
import com.team2502.robot2019.subsystem.DrivetrainSubsystem;
import com.team2502.robot2019.subsystem.solenoid.ClimbClawSolenoid;
import com.team2502.robot2019.subsystem.solenoid.HatchIntakeSolenoid;
import com.team2502.robot2019.subsystem.solenoid.OBASolenoid;
import com.team2502.robot2019.utils.ScoringHUD;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
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
    public static MainActionScheduler ACTION_SCHEDULER = new MainActionScheduler(RealClock.CLOCK);
    public static DrivetrainSubsystem DRIVE_TRAIN;
    public static HatchIntakeSolenoid HATCH_INTAKE;
    public static ClimbClawSolenoid CLIMB_CLAWS;
    public static OBASolenoid OBA;
    public static CargoSubsystem CARGO_ACTIVE;
    public static ClimberSubsystem CLIMBER;
    public static CrawlerSubsystem CRAWLER;
    public static Compressor COMPRESSOR;
//    public static UsbCamera CAMERA0;
//    public static UsbCamera CAMERA1;
//    public static UsbCamera CAMERA2;
//    private HttpCamera VISION_CAM;
//
//    public static VideoSink SERVER;
    public static ScoringHUD SCORING_HUD;
    public static  NetworkTable VISION_TABLE;
    public static NetworkTableEntry tvecs1Entry;
    public static NetworkTableEntry tvecs2Entry;
    public static NetworkTableEntry angleEntry;
    public static NetworkTableEntry connectedEntry;
    public static NetworkTableEntry seesTarget;

    public static List<Runnable> onDisableThings = new ArrayList<>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        //CAMERA0 = CameraServer.getInstance().startAutomaticCapture(0);
        //CAMERA1 = CameraServer.getInstance().startAutomaticCapture(1);
        //VISION_CAM = new HttpCamera("vision", "http://frcvision.local:1181/?action=stream", HttpCamera.HttpCameraKind.kCSCore);// + Constants.Autonomous.COPROCESSOR_MDNS_ADDR + ":1181");
        //CameraServer.getInstance().addCamera(VISION_CAM);
        //SERVER = CameraServer.getInstance().getServer();

        //SERVER.setSource(VISION_CAM);


        DRIVE_TRAIN = new DrivetrainSubsystem();
        HATCH_INTAKE = new HatchIntakeSolenoid();
        CLIMB_CLAWS = new ClimbClawSolenoid();
        CARGO_ACTIVE = new CargoSubsystem();
        CLIMBER = new ClimberSubsystem();
        COMPRESSOR = new Compressor();
        CRAWLER = new CrawlerSubsystem();
        OBA = new OBASolenoid();

        SCORING_HUD = new ScoringHUD();
        AutoSwitcher.putToSmartDashboard();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        VISION_TABLE = inst.getTable("SmartDashboard");
        tvecs1Entry = VISION_TABLE.getEntry("tvecs1");
        tvecs1Entry.setDouble(-9001);
        tvecs2Entry = VISION_TABLE.getEntry("tvecs2");
        tvecs2Entry.setDouble(-9001);
        angleEntry = VISION_TABLE.getEntry("angle");
        angleEntry.setDouble(-9001);
        connectedEntry = VISION_TABLE.getEntry("connected");
        connectedEntry.setNumber(0);
        seesTarget = VISION_TABLE.getEntry("seesTarget");
        seesTarget.setBoolean(true);

        SmartDashboard.putNumber("pleaseUnstick", 1);

        ActionGroup defaultActions = new ActionGroup()
                .addParallel(new BackgroundAction(5, TimeUnit.MILLISECONDS, Robot.DRIVE_TRAIN::update));
        ACTION_SCHEDULER.scheduleAction(defaultActions);

        SmartDashboard.putNumber("kP", 0.2);
        SmartDashboard.putNumber("kI", 0);
        SmartDashboard.putNumber("kD", 0);

        SmartDashboard.putNumber(Constants.ShuffleboardKeys.DELAY_AUTO, 0);

        MathUtils.init();

        SmartDashboard.putNumber("minDistance", 1);
        SmartDashboard.putNumber("maxDistance", 5);
        SmartDashboard.putNumber("minSpeed", 3);
        SmartDashboard.putNumber("maxSpeed", 10);
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
        Scheduler.getInstance().run(); // Runs Teleop when enabled, disabled when disabled
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
//        Robot.DRIVE_TRAIN.getPigeon().setFusedHeading(0);
//        Robot.DRIVE_TRAIN.getLocEstimator().reset();
//        Scheduler.getInstance().add(AutoSwitcher.getAutoInstance());
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
        PurePursuitAction pp = new PurePursuitAction(10, TimeUnit.MILLISECONDS, ppMoveStrat, DRIVE_TRAIN.getLocEstimator(), Constants.Autonomous.getLookaheadBounds(DRIVE_TRAIN), DRIVE_TRAIN);
        BackgroundAction loc = new BackgroundAction(5, TimeUnit.MILLISECONDS, DRIVE_TRAIN::update);
        ActionGroup group = new ActionGroup();
        group.with(loc);
        group.addSequential(pp);
        return new CommandCreator(group, Robot.ACTION_SCHEDULER);
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
        free();
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
        OBA.set(false);
        HATCH_INTAKE.set(false);
        CLIMB_CLAWS.set(false);

        onDisableThings.forEach(Runnable::run);
        onDisableThings.clear();
    }

    @Override
    public void disabledPeriodic() { }
}
