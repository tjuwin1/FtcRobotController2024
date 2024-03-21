package org.firstinspires.ftc.teamcode.helpers;

public class CurvePoint {
    public double xPos;
    public double yPos;
    public double angle;

    public CurvePoint(double x,double y,double ang){
        this.xPos = x;
        this.yPos = y;
        this.angle = ang;
    }
    public CurvePoint(double x,double y){
        this.xPos = x;
        this.yPos = y;
    }
}
