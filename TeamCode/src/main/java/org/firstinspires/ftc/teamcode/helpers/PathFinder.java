package org.firstinspires.ftc.teamcode.helpers;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Iterator;

public class PathFinder {

    public static double closestDistance(ArrayList<CurvePoint> allPoints, CurvePoint robotLocation){
        double closestDist = 999;
        for(int i=0;i< allPoints.size() - 1;i++) {
            double slope = 0, intercept = 0;
            CurvePoint start = allPoints.get(i);
            CurvePoint end = allPoints.get(i + 1);

            //y = mx + b
            //if X is equal, then y = c
            if (start.xPos == end.xPos) {
                slope = 0;
            } else {
                slope = (end.yPos - start.yPos) / (end.xPos - start.xPos);
            }
            intercept = end.yPos - slope * end.xPos;

            //The distance between point P=(x0 ,y0) and line L:ax+by+c=0 is
            //      |ax0 + by0 + c|
            // d =  --------------
            //         √(a²+b²)

            double distance = Math.abs(slope * robotLocation.xPos - robotLocation.yPos + intercept) / Math.hypot(slope, 1);
            if (closestDist > distance) { //Keep a minimum radius of n-inches for smooth turning
                closestDist = distance;
                if(closestDist < Driver.TURN_AT_DISTANCE){
                    closestDist = Driver.TURN_AT_DISTANCE;
                }
            }
        }
        return closestDist;
    }

    public static CurvePoint getFollowPoint(ArrayList<CurvePoint> pathPoints, CurvePoint robotLocation) {
        double followRadius = closestDistance(pathPoints,robotLocation);
        CurvePoint endPoint = pathPoints.get(pathPoints.size() - 1);
        double distanceToEnd = Math.hypot(robotLocation.yPos - endPoint.yPos, robotLocation.xPos - endPoint.xPos);
        double closestDistance = distanceToEnd;
        CurvePoint followMe = pathPoints.get(0);

        for(int i = 0; i < pathPoints.size() - 1; ++i) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);
            ArrayList<Point> intersections = MathFunctions.lineCircleIntersection(robotLocation.toPoint(), followRadius, startLine.toPoint(), endLine.toPoint());
            Iterator var11 = intersections.iterator();

            while(var11.hasNext()) {
                Point thisIntersection = (Point)var11.next();
                double distanceToIntersection = Math.hypot(thisIntersection.y - endPoint.yPos, thisIntersection.x - endPoint.xPos);
                if(distanceToIntersection < closestDistance){
                    followMe.xPos = thisIntersection.x;
                    followMe.yPos = thisIntersection.y;
                    followMe.angle = endLine.angle;
                }
            }
        }

        return followMe;
    }
}
