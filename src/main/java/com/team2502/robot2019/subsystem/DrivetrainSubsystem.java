package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.github.ezauton.core.action.require.BaseResource;
import com.github.ezauton.core.actuators.VelocityMotor;
import com.github.ezauton.core.localization.RotationalLocationEstimator;
import com.github.ezauton.core.localization.Updateable;
import com.github.ezauton.core.localization.UpdateableGroup;
import com.github.ezauton.core.localization.estimators.EncoderRotationEstimator;
import com.github.ezauton.core.localization.sensors.Encoders;
import com.github.ezauton.core.localization.sensors.TranslationalDistanceSensor;
import com.github.ezauton.core.localization.sensors.VelocityEstimator;
import com.github.ezauton.core.robot.implemented.TankRobotTransLocDriveable;
import com.github.ezauton.core.trajectory.geometry.ImmutableVector;
import com.github.ezauton.core.utils.MathUtils;
import com.github.ezauton.core.utils.RealClock;
import com.github.ezauton.core.utils.Stopwatch;
import com.github.ezauton.wpilib.motors.TypicalMotor;
import com.team2502.robot2019.*;
import com.team2502.robot2019.command.teleop.DriveCommand;
import com.team2502.robot2019.subsystem.interfaces.DriveTrain;
import com.team2502.robot2019.utils.IPIDTunable;
import com.team2502.robot2019.utils.Utils;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;

public class DrivetrainSubsystem extends Subsystem implements IPIDTunable, DriveTrain, Updateable, DashboardData.DashboardUpdater
{
    private final WPI_TalonSRX backLeft;
    private final WPI_TalonSRX backRight;

    public final WPI_TalonSRX frontLeft;
    public final WPI_TalonSRX frontRight;

    private final TypicalMotor left;
    private final TypicalMotor right;

    private final TankRobotTransLocDriveable trtls;
    private final EncoderRotationEstimator locEst;
    private final RotationalLocationEstimator rotEst;
    private final VelocityEstimator velEst;

    private SendableChooser<TeleopMode> teleopChooser;

    private final DifferentialDrive diffDrive;

    private final PigeonIMU pigeonIMU;


    private double kP = Constants.Physical.DriveTrain.DEFAULT_KP;
    private double kI = Constants.Physical.DriveTrain.DEFAULT_KI;
    private double kD = Constants.Physical.DriveTrain.DEFAULT_KD;
    private double kF = Constants.Physical.DriveTrain.DEFAULT_KF_RIGHT_COMPBOT;

    private boolean forward = true;

    private final BaseResource resource = new BaseResource();
    private final UpdateableGroup updateableGroup;
    private final TranslationalDistanceSensor leftSensor;
    private final TranslationalDistanceSensor rightSensor;

    // for veldrive
    private final Stopwatch stopwatch;
    private double lastLeftVelTarget = Double.NaN;
    private double lastRightVelTarget = Double.NaN;

    public DrivetrainSubsystem()
    {
        stopwatch = new Stopwatch(RealClock.CLOCK);
        stopwatch.init();

        backLeft = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_BACK_LEFT);
        backRight = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_BACK_RIGHT);
        frontLeft = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_FRONT_LEFT);
        frontRight = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_FRONT_RIGHT);

        pigeonIMU = new PigeonIMU(backLeft);

        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
        frontLeft.setSensorPhase(true);

        frontLeft.configClosedloopRamp(0);
        frontRight.configClosedloopRamp(0);

        frontLeft.configOpenloopRamp(Constants.Physical.DriveTrain.SECONDS_FROM_NEUTRAL_TO_FULL);
        frontRight.configOpenloopRamp(Constants.Physical.DriveTrain.SECONDS_FROM_NEUTRAL_TO_FULL);

        SpeedControllerGroup scgLeft = new SpeedControllerGroup(frontLeft, backLeft);
        SpeedControllerGroup scgRight = new SpeedControllerGroup(frontRight, backRight);

        scgLeft.setInverted(false);
        scgRight.setInverted(true);

        diffDrive = new DifferentialDrive(scgLeft, scgRight);

        right = new TypicalMotor()
        {
            public void runVelocity(double targetVelocity)
            {
                this.makeSlavesFollowMaster();
                frontRight.set(ControlMode.Velocity, targetVelocity / Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS);
            }

            public void runVoltage(double targetVoltage)
            {
                this.makeSlavesFollowMaster();
                frontRight.set(ControlMode.PercentOutput, targetVoltage);
            }

            public double getPosition()
            {
                return (double) frontRight.getSelectedSensorPosition(0) * Constants.Physical.DriveTrain.ENC_UNITS_TO_FEET;
            }

            public double getVelocity()
            {
                return (double) frontRight.getSelectedSensorVelocity(0) * Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS;
            }

            private void makeSlavesFollowMaster()
            {
                backRight.follow(frontRight);
            }
        };


        left = new TypicalMotor()
        {
            public void runVelocity(double targetVelocity)
            {
                this.makeSlavesFollowMaster();
                frontLeft.set(ControlMode.Velocity, targetVelocity / Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS);
            }

            public void runVoltage(double targetVoltage)
            {
                this.makeSlavesFollowMaster();
                frontLeft.set(ControlMode.PercentOutput, targetVoltage);
            }

            public double getPosition()
            {
                return (double) frontLeft.getSelectedSensorPosition(0) * Constants.Physical.DriveTrain.ENC_UNITS_TO_FEET;
            }

            public double getVelocity()
            {
                return (double) frontLeft.getSelectedSensorVelocity(0) * Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS;
            }

            private void makeSlavesFollowMaster()
            {
                backLeft.follow(frontLeft);
            }
        };

        leftSensor = Encoders.toTranslationalDistanceSensor(
                1,
                1,
                left
                                                           );
        rightSensor = Encoders.toTranslationalDistanceSensor(
                1,
                1,
                right
                                                            );

        pigeonIMU.setYaw(0);
        rotEst = () -> {
            double[] ypr = new double[3];
            pigeonIMU.getYawPitchRoll(ypr);
            return -MathUtils.Kinematics.navXToRad(ypr[0]);
        };
        velEst = () -> (leftSensor.getVelocity() + rightSensor.getVelocity()) / 2;

        locEst = new EncoderRotationEstimator(rotEst, new TranslationalDistanceSensor()
        {
            @Override
            public double getPosition()
            {
                return (leftSensor.getPosition() + rightSensor.getPosition()) / 2;
            }

            @Override
            public double getVelocity()
            {
                return (leftSensor.getVelocity() + rightSensor.getVelocity()) / 2;
            }
        });
//        locEst = new TankRobotEncoderEncoderEstimator(leftSensor, rightSensor, Constants.Physical.DriveTrain.TANK_ROBOT_CONSTANTS);
        locEst.reset();


        trtls = new TankRobotTransLocDriveable(left, right, locEst, rotEst, Constants.Physical.DriveTrain.TANK_ROBOT_CONSTANTS);

        // DO NOT CHANGE
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);

        updateableGroup = new UpdateableGroup(locEst);

        DashboardData.addUpdater(this);

        applyPID();
        frontRight.config_kF(0, Constants.Physical.DriveTrain.DEFAULT_KF_RIGHT_PRACTICE, Constants.INIT_TIMEOUT);
        frontLeft.config_kF(0, Constants.Physical.DriveTrain.DEFAULT_KF_LEFT_PRACTICE, Constants.INIT_TIMEOUT);

        teleopChooser = new SendableChooser<>();

        for(int i = 0; i < TeleopMode.values().length; i++)
        {
            TeleopMode mode = TeleopMode.values()[i];
            if(i == 0) { teleopChooser.addDefault(mode.name, mode); }
            else { teleopChooser.addObject(mode.name, mode); }
        }

        SmartDashboard.putData("Teleop Mode", teleopChooser);
    }

    public enum TeleopMode
    {
        // The first item in this enum is the default
        TANK_VOLTAGE("Tank: Voltage"),
        TANK_VELOCITY("Tank: Velocity"),
        ARCADE("Arcade"),
        CURVATURE("Curvature");

        String name;

        TeleopMode(String name)
        {
            this.name = name;
        }
    }

    /**
     * Runs the drive train motors with the specified control mode, at the specified speeds for each side.
     *
     * @param controlMode the control mode to use.
     * @param leftVal     value/speed for left side of DT.
     * @param rightVal    value/speed for right side of DT.
     * @see com.ctre.phoenix.motorcontrol
     */
    public void runMotors(ControlMode controlMode, double leftVal, double rightVal)
    {
        frontLeft.set(controlMode, leftVal);
        frontRight.set(controlMode, rightVal);
    }

    /**
     * Runs the drive train in velocity mode (values are native encoder units per 100ms)
     *
     * @param leftVal  speed of left side of DT in FT/SEC.
     * @param rightVal speed of right side of DT in FT/SEC.
     */
    public void runMotorsVelocity(double leftVal, double rightVal)
    {
        left.runVelocity(leftVal);
        right.runVelocity(rightVal);
    }

    public void runMotorsPosition(double leftPos, double rightPos) {
        double leftPosEncUnits = leftPos / Constants.Physical.DriveTrain.ENC_UNITS_TO_FEET;
        double rightPosEncUnits = rightPos / Constants.Physical.DriveTrain.ENC_UNITS_TO_FEET;

        frontLeft.configAllowableClosedloopError(0, 64);
        frontRight.configAllowableClosedloopError(0, 64);

        frontLeft.configMotionAcceleration((int) (Constants.Physical.DriveTrain.MAX_FPS2_ACCEL / Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS));
        frontLeft.configMotionCruiseVelocity((int) (10 / Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS));

        frontRight.configMotionAcceleration((int) (Constants.Physical.DriveTrain.MAX_FPS2_ACCEL / Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS));
        frontRight.configMotionCruiseVelocity((int) (10 / Constants.Physical.DriveTrain.ENC_UNITS_TO_FPS));


        frontLeft.set(ControlMode.MotionMagic, leftPosEncUnits);
        frontRight.set(ControlMode.MotionMagic, rightPosEncUnits);
    }
    @Deprecated
    public void runMotorsVoltage(double leftVolts, double rightVolts)
    {
        left.runVoltage(leftVolts);
        right.runVoltage(rightVolts);
    }

    @Override
    public VelocityMotor getLeft()
    {
        return left;
    }

    @Override
    public VelocityMotor getRight()
    {
        return right;
    }

    @Override
    public EncoderRotationEstimator getLocEstimator()
    {
        return locEst;
    }

    @Override
    public RotationalLocationEstimator getRotEstimator()
    {
        return rotEst;
    }

    @Override
    public VelocityEstimator getVelocityEstimator()
    {
        return velEst;
    }

    public WPI_TalonSRX getFrontLeft()
    {
        return frontLeft;
    }

    public WPI_TalonSRX getFrontRight()
    {
        return frontRight;
    }

    public double getAngularVelocity() {
        double[] xyz_degrees_per_sec = new double[3];
        pigeonIMU.getRawGyro(xyz_degrees_per_sec);
        return xyz_degrees_per_sec[2];
    }

    @Override
    public double getkP()
    {
        return kP;
    }

    @Override
    public void setkP(double kP)
    {
        this.kP = kP;
        applyPID();
    }

    @Override
    public double getkI()
    {
        return kI;
    }

    @Override
    public void setkI(double kI)
    {
        this.kI = kI;
        applyPID();
    }

    @Override
    public double getkD()
    {
        return kD;
    }

    @Override
    public void setkD(double kD)
    {
        this.kD = kD;
        applyPID();
    }

    @Override
    public double getkF()
    {
        return kF;
    }

    @Override
    public void setkF(double kF)
    {
        this.kF = kF;
        applyPID();
    }

    @Override
    public void setPID(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        applyPID();
    }

    public void applyPID()
    {
        frontLeft.selectProfileSlot(0, 0);
        frontRight.selectProfileSlot(0, 0);

        frontLeft.config_kP(0, kP, Constants.INIT_TIMEOUT);
        frontLeft.config_kI(0, kI, Constants.INIT_TIMEOUT);
        frontLeft.config_kD(0, kD, Constants.INIT_TIMEOUT);
//        frontLeft.config_kF(0, kF, Constants.INIT_TIMEOUT);

        frontRight.config_kP(0, kP, Constants.INIT_TIMEOUT);
        frontRight.config_kI(0, kI, Constants.INIT_TIMEOUT);
        frontRight.config_kD(0, kD, Constants.INIT_TIMEOUT);

    }

    @Override
    public boolean driveTowardTransLoc(double speed, ImmutableVector loc)
    {
        return trtls.driveTowardTransLoc(speed, loc);
    }

    @Override
    public boolean driveSpeed(double speed)
    {
        return trtls.driveSpeed(speed);
    }


    @Override
    public BaseResource getResource()
    {
        return resource;
    }

    @Override
    public boolean update()
    {
        return updateableGroup.update();
    }

    public boolean isForward()
    {
        return forward;
    }

    public void setForward(boolean forward)
    {
        this.forward = forward;
    }

    public void teleopDrive()
    {
        double speed1, speed2;
        switch(teleopChooser.getSelected())
        {
            case ARCADE:
                speed1 = -OI.JOYSTICK_DRIVE_RIGHT.getY();
                speed2 = OI.JOYSTICK_DRIVE_RIGHT.getTwist();
                teleopDriveArcade(speed1, speed2);
                break;

            case TANK_VOLTAGE:
                speed1 = -OI.JOYSTICK_DRIVE_LEFT.getY();
                speed2 = -OI.JOYSTICK_DRIVE_RIGHT.getY();
                teleopDriveTankVoltage(speed1, speed2);
                break;

            case TANK_VELOCITY:
                speed1 = -OI.JOYSTICK_DRIVE_LEFT.getY();
                speed2 = -OI.JOYSTICK_DRIVE_RIGHT.getY();
                teleopDriveTankVelocity(speed1, speed2);
                break;

            case CURVATURE:
                double left = -OI.JOYSTICK_DRIVE_LEFT.getY();
                double right = -OI.JOYSTICK_DRIVE_RIGHT.getY();
                boolean isQuickturn = Math.abs(left - right) > 1.0;

                speed1 = (left + right) / 2;
                speed2 = left - right;

                teleopDriveCurvature(speed1, speed2, isQuickturn);
                break;

            default:
                speed1 = -OI.JOYSTICK_DRIVE_LEFT.getY();
                speed2 = -OI.JOYSTICK_DRIVE_RIGHT.getY();
                teleopDriveTankVoltage(speed1, speed2);
        }
    }

    private void teleopDriveCurvature(double xSpeed, double zRotation, boolean isQuickTurn)
    {
        diffDrive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
    }

    private void teleopDriveArcade(double xSpeed, double zRotation)
    {
        diffDrive.arcadeDrive(xSpeed, zRotation);
    }

    private void teleopDriveTankVoltage(double leftSpeed, double rightSpeed)
    {
        diffDrive.tankDrive(leftSpeed, rightSpeed);
    }

    private void teleopDriveTankVelocity(double leftVolts, double rightVolts)
    {
        // skipping input squaring
        double leftVelTarget = leftVolts * Constants.Physical.DriveTrain.MAX_FPS_SPEED;
        double rightVelTarget = rightVolts * Constants.Physical.DriveTrain.MAX_FPS_SPEED;

        if(!Double.isNaN(lastLeftVelTarget) && !Double.isNaN(lastRightVelTarget)) {
            double dt = stopwatch.pop(TimeUnit.SECONDS);


            leftVelTarget = Utils.handleMaxAcc(leftVelTarget, lastLeftVelTarget, dt);
            rightVelTarget = Utils.handleMaxAcc(rightVelTarget, lastRightVelTarget, dt);
        }

        if (Math.abs(leftVelTarget) > Constants.Teleop.JOYSTICK_DEADBAND)
        {
            runMotorsVelocity(leftVelTarget, rightVelTarget);
        }

        else
        {
            runMotorsVoltage(0.0D, 0.0D);
        }

        lastLeftVelTarget = leftVelTarget;
        lastRightVelTarget = rightVelTarget;
    }

    @Override
    public void updateDashboard()
    {
        update();
        SmartDashboard.putNumber("rot", rotEst.estimateHeading());
        SmartDashboard.putNumber("left vel", leftSensor.getVelocity());
        SmartDashboard.putNumber("right vel", rightSensor.getVelocity());
        SmartDashboard.putNumber("left pos", frontLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("right pos", frontRight.getSelectedSensorPosition());
        SmartDashboard.putString("loc", locEst.estimateLocation().toString());
    }

    protected void initDefaultCommand()
    {
        setDefaultCommand(new DriveCommand());
    }

    public void resetEncoderPosition() {
        frontLeft.setSelectedSensorPosition(0);
        frontRight.setSelectedSensorPosition(0);
    }
}
