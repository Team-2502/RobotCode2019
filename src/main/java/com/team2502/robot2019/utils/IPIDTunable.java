package com.team2502.robot2019.utils;

public interface IPIDTunable {
    double getkP();


    void setkP(double kP);


    double getkI();


    void setkI(double kI);


    double getkD();


    void setkD(double kD);


    double getkF();


    void setkF(double kF);


    void setPID(double kP, double kI, double kD);
}
