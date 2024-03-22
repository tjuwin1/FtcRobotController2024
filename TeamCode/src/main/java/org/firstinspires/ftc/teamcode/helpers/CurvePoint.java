package org.firstinspires.ftc.teamcode.helpers;

import org.checkerframework.checker.units.qual.C;

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

    protected static CurvePoint subtract(CurvePoint point1,CurvePoint point2){
        return new CurvePoint(point2.xPos - point1.xPos,
                point2.yPos - point1.yPos,
                MathFunctions.AngleWrap(point2.angle - point1.angle));
    }

    protected static CurvePoint add(CurvePoint point1,CurvePoint point2){
        return new CurvePoint(point2.xPos + point1.xPos,
                point2.yPos + point1.yPos,
                MathFunctions.AngleWrap(point2.angle + point1.angle));
    }

    protected static CurvePoint multiply(CurvePoint point,double multiplier){
        return new CurvePoint(point.xPos * multiplier,
                point.yPos * multiplier,
                MathFunctions.AngleWrap(point.angle * multiplier));
    }
}
