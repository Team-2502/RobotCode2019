package com.team2502.robot2019.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.team2502.robot2019.Constants;
import com.team2502.robot2019.RobotMap;
import com.team2502.robot2019.command.teleop.DriveCommand;
import com.team2502.robot2019.utils.IPIDTunable;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DrivetrainSubsystem extends Subsystem implements IPIDTunable {

    private final WPI_TalonSRX backLeft;
    private final WPI_TalonSRX backRight;

    private final WPI_TalonSRX frontLeft;
    private final WPI_TalonSRX frontRight;

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    public DrivetrainSubsystem() {
        backLeft = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_BACK_LEFT);
        backRight = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_BACK_RIGHT);
        frontLeft = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_FRONT_LEFT);
        frontRight = new WPI_TalonSRX(RobotMap.Motor.DRIVE_TRAIN_FRONT_RIGHT);

        // DO NOT CHANGE
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveCommand()); //TODO: DriveCommand
    }

    public void runMotors(ControlMode controlMode, double leftVal, double rightVal) {
        frontLeft.set(controlMode, leftVal);
        frontRight.set(controlMode, rightVal);
    }

    public void runMotorsVelocity(double leftVal, double rightVal) {
        runMotors(ControlMode.Velocity, leftVal, rightVal); //TODO: feet per second -> native units per 100 ms conversion
    }

    @Deprecated
    public void runMotorsVoltage(double leftVolts, double rightVolts) {
        runMotors(ControlMode.PercentOutput, leftVolts, rightVolts);
    }

    public WPI_TalonSRX getFrontLeft() {
        return frontLeft;
    }

    public WPI_TalonSRX getFrontRight() {
        return frontRight;
    }

    @Override
    public double getkP() {
        return kP;
    }

    @Override
    public void setkP(double kP) {
        this.kP = kP;
        applyPID();
    }

    @Override
    public double getkI() {
        return kI;
    }

    @Override
    public void setkI(double kI) {
        this.kI = kI;
        applyPID();
    }

    @Override
    public double getkD() {
        return kD;
    }

    @Override
    public void setkD(double kD) {
        this.kD = kD;
        applyPID();
    }

    @Override
    public double getkF() {
        return kF;
    }

    @Override
    public void setkF(double kF) {
        this.kF = kF;
        applyPID();
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        applyPID();
    }

    public void applyPID() {
        frontLeft.config_kP(0, kP, Constants.INIT_TIMEOUT);
        frontLeft.config_kI(0, kI, Constants.INIT_TIMEOUT);
        frontLeft.config_kD(0, kD, Constants.INIT_TIMEOUT);
        frontLeft.config_kF(0, kF, Constants.INIT_TIMEOUT);

        frontRight.config_kP(0, kP, Constants.INIT_TIMEOUT);
        frontRight.config_kI(0, kI, Constants.INIT_TIMEOUT);
        frontRight.config_kD(0, kD, Constants.INIT_TIMEOUT);
        frontRight.config_kF(0, kF, Constants.INIT_TIMEOUT);
    }
}
