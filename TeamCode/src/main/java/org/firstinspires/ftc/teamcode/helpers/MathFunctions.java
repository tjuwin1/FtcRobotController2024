package org.firstinspires.ftc.teamcode.helpers;

import java.util.ArrayList;
import org.opencv.core.Point;

public class MathFunctions {
    public MathFunctions() {
    }

    public static double AngleInPositiveX(double angle){
        //Get an angle between -90° and 90°
        while(angle < -Math.PI/2) {
            angle += (Math.PI);
        }

        while(angle > Math.PI /2) {
            angle -= (Math.PI);
        }

        return angle;
    }

    public static double AngleBetweenPoints(double xDiff, double yDiff){
        if(xDiff == 0){
            if(yDiff < 0){ //Negative slope
                return - Math.PI / 2;
            }else{
                return Math.PI / 2;
            }
        } else if (yDiff == 0) {
            if(xDiff > 0){
                return 0;
            }else{
                return Math.PI;
            }
        }
        return Math.atan2(yDiff, xDiff);
    }

    public static double AngleWrap(double angle) {
        //Get an angle between -180° and 180°
        while(angle < -Math.PI) {
            angle += (Math.PI * 2);
        }

        while(angle > Math.PI) {
            angle -= (Math.PI * 2);
        }

        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }

        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        double quadraticA = 1.0 + Math.pow(m1, 2.0);
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;
        double quadraticB = 2.0 * m1 * y1 - 2.0 * Math.pow(m1, 2.0) * x1;
        double quadraticC = Math.pow(m1, 2.0) * Math.pow(x1, 2.0) - 2.0 * y1 * m1 * x1 + Math.pow(y1, 2.0) - Math.pow(radius, 2.0);
        ArrayList<Point> allPoints = new ArrayList();

        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2.0) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2.0) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;
            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch (Exception var30) {
        }

        return allPoints;
    }
}
