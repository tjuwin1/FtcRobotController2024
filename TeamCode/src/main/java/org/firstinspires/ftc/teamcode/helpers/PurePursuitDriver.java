package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Optional;

public class PurePursuitDriver {
    private Hardware robot;

    private static String TAG = "PathFollower";
    private static final double TURN_AT_DISTANCE = 15;

    private static final double STUCK_CHECK_TIME = 10;
    private final double MIN_ANGLE = Math.toRadians(0.5);
    private final double MIN_DISTANCE = 0.5;
    private final double MIN_CHANGE = 0.05;
    private final double MIN_POWER = 0.1; //Minimum required power for the robot to move

    //Values for Proportional in PID tuning
    private static final double kPxy=0.12, kP0=1;
    private static final double kDxy=0.012,kD0=0.045;
    private static final double kIxy=0.0012, kI0=0.0045;

    private LinearOpMode opMode;
    private DataFileLogger logger;
    private ElapsedTime runtime = new ElapsedTime();
    private LogOutput logOutput;

    private class robotPath{
        public double timestamp;
        public CurvePoint pose;
        public robotPath(CurvePoint pose){
            timestamp = runtime.seconds();
            this.pose = pose;
        }
    }

    private ArrayList<robotPath> prevDeltas = new ArrayList<robotPath>();

    public PurePursuitDriver(LinearOpMode opMode,Hardware robot, LogOutput logOutput){
        this.logger = new DataFileLogger(TAG,false);
        this.opMode = opMode;
        this.robot = robot;
        this.logOutput = logOutput;

        logger.addField(
                "cycleTime","xPos", "yPos", "angle","tgtX","tgtY","tgta",
                "distanceToTarget", "absoluteAngleToTarget",
                "relativeAngleToPoint", "relativeTurnAngle", "xyTotal", "xPower", "yPower",
                "dxPos", "dyPos", "dangle", "xPIDPower", "yPIDPower",
                "aPIDPower", "maxPower","lfPower", "lbPower", "rfPower", "rbPower", "\n");
    }

    private boolean mustContinue(){
        return this.opMode.opModeIsActive();
    }

    public void stopReleaseAll(){
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }

    public void close(){
        logger.closeLog();
    }

    public void followPath(ArrayList<CurvePoint> allPoints){
        if(!this.mustContinue()) {
            return;
        }

        Boolean firstRun = false;
        CurvePoint prevPoint = new CurvePoint(0, 0, 0);
        CurvePoint deltaError = new CurvePoint(0, 0, 0);
        CurvePoint integralPoint = new CurvePoint(0, 0, 0);

        prevDeltas.clear();
        runtime.reset();

        while (this.mustContinue()) {
            robot.localizer.updateReadings();

            double cycleTime = runtime.seconds();
            runtime.reset();

            CurvePoint point = PathFinder.getFollowPoint(allPoints,robot.localizer.robotPose);

            //Find the maximum allowed power change in a second to limit acceleration
            double powerVariation = Range.clip(cycleTime * 10,0.1,0.3);
            double lfPowerMax = Range.clip(Math.abs(robot.leftFrontDrive.getPower()) + powerVariation, 0,1);
            double lbPowerMax = Range.clip(Math.abs(robot.leftBackDrive.getPower()) + powerVariation, 0,1);
            double rfPowerMax = Range.clip(Math.abs(robot.rightFrontDrive.getPower()) + powerVariation, 0,1);
            double rbPowerMax = Range.clip(Math.abs(robot.rightBackDrive.getPower()) + powerVariation, 0,1);

            double distanceToTarget = Math.hypot(point.xPos - robot.localizer.robotPose.xPos, point.yPos - robot.localizer.robotPose.yPos);
            double absoluteAngleToTarget = Math.atan2(point.yPos - robot.localizer.robotPose.yPos, point.xPos - robot.localizer.robotPose.xPos);
            double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - robot.localizer.robotPose.angle);
            double xPower = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double yPower = Math.sin(relativeAngleToPoint) * distanceToTarget;
            double relativeTurnAngle = Range.clip(MathFunctions.AngleWrap(point.angle - robot.localizer.robotPose.angle), -Math.PI / 2, Math.PI / 2);
            if (distanceToTarget > TURN_AT_DISTANCE){ //If distance to target is more than 15 inches, then turn towards the target to make it easier to drive
                relativeTurnAngle = Range.clip(absoluteAngleToTarget, -Math.PI / 2, Math.PI / 2);
            }

            if (!firstRun) {
                deltaError = CurvePoint.subtract(prevPoint, new CurvePoint(xPower, yPower, relativeTurnAngle));
                integralPoint = CurvePoint.add(integralPoint, CurvePoint.multiply(deltaError, cycleTime));
                addDelta(deltaError);
            }

            if (Math.abs(relativeTurnAngle) <= MIN_ANGLE && distanceToTarget <= MIN_DISTANCE) {
                stopReleaseAll();
                logger.addField(String.format("\n----------------Reached--Angle [%.5f <= %.5f] and Distance [%.5f <= %.5f]------------------------\n",
                        Math.toDegrees(Math.abs(relativeTurnAngle)),Math.toDegrees(MIN_ANGLE),distanceToTarget,MIN_DISTANCE));
                return;
            } else if (!amImoving()) {
                stopReleaseAll();
                logger.addField(String.format("\n----------------Stuck--Angle [%.5f <= %.5f] and Distance [%.5f <= %.5f]------------------------\n",
                        Math.toDegrees(Math.abs(relativeTurnAngle)),Math.toDegrees(MIN_ANGLE),distanceToTarget,MIN_DISTANCE));
                return;
            }

            //PID Controller
            double xPIDPower = (kPxy * xPower) + (kIxy * integralPoint.xPos) + (kDxy * deltaError.xPos / cycleTime);
            double yPIDPower = (kPxy * yPower) + (kIxy * integralPoint.yPos) + (kDxy * deltaError.yPos / cycleTime);
            double aPIDPower = (kP0 * Math.sin(relativeTurnAngle)) + (kI0 * Math.sin(integralPoint.angle)) + (kD0 * Math.sin(deltaError.angle) / cycleTime);

            prevPoint.xPos = xPower;
            prevPoint.yPos = yPower;
            prevPoint.angle = relativeTurnAngle;

            double xyTotal = Math.abs(xPIDPower) + Math.abs(yPIDPower) + Math.abs(aPIDPower);
            if (xyTotal > 1) {
                xPIDPower /= xyTotal;
                yPIDPower /= xyTotal;
                aPIDPower /= xyTotal;
            }

            //Set motor powers
            double lfPower = Range.clip(xPIDPower - yPIDPower - aPIDPower, -lfPowerMax, lfPowerMax); //(y + x + rx)
            double lbPower = Range.clip(xPIDPower + yPIDPower - aPIDPower, -lbPowerMax, lbPowerMax); //(y - x + rx)
            double rfPower = Range.clip(xPIDPower + yPIDPower + aPIDPower, -rfPowerMax, rfPowerMax); //(y - x - rx)
            double rbPower = Range.clip(xPIDPower - yPIDPower + aPIDPower, -rbPowerMax, rbPowerMax); //(y + x - rx)

            double maxPower = MathFunctions.maxNumber(Math.abs(lfPower),Math.abs(lbPower),Math.abs(rfPower),Math.abs(rbPower));
            if(maxPower < MIN_POWER){ //Minimum required power for the robot to move
                lfPower *= MIN_POWER / maxPower;
                lbPower *= MIN_POWER / maxPower;
                rfPower *= MIN_POWER / maxPower;
                rbPower *= MIN_POWER / maxPower;
            }

            robot.leftFrontDrive.setPower(lfPower);
            robot.leftBackDrive.setPower(lbPower);
            robot.rightFrontDrive.setPower(rfPower);
            robot.rightBackDrive.setPower(rbPower);

            logger.addField(cycleTime,
                    robot.localizer.robotPose.xPos, robot.localizer.robotPose.yPos, Math.toDegrees(robot.localizer.robotPose.angle),
                    point.xPos,point.yPos,Math.toDegrees(point.angle),
                    distanceToTarget, Math.toDegrees(absoluteAngleToTarget),
                    Math.toDegrees(relativeAngleToPoint), Math.toDegrees(relativeTurnAngle), xyTotal, xPower, yPower,
                    deltaError.xPos, deltaError.yPos, Math.toDegrees(deltaError.angle),
                    xPIDPower, yPIDPower, aPIDPower, maxPower, lfPower, lbPower, rfPower, rbPower);

            firstRun = false;
            this.logOutput.update();
        }
    }

    private void addDelta(CurvePoint deltaError){
        prevDeltas.add(0,new robotPath(deltaError));
    }

    private boolean amImoving(){
        boolean enoughDataExists = false;
        CurvePoint sumErrs = new CurvePoint(0, 0, 0);

        for (int cnt = 0; cnt < prevDeltas.size(); cnt++) {
            robotPath deltaPose =  prevDeltas.get(cnt);
            if (deltaPose.timestamp < STUCK_CHECK_TIME) {
                sumErrs = CurvePoint.add(sumErrs, deltaPose.pose);
            }else{
                enoughDataExists = true; //More than stuck check time is available
                prevDeltas.subList(cnt+1, prevDeltas.size()).clear();
                break;
            }
        }

        if(enoughDataExists) {
            if (Math.abs(sumErrs.xPos) < MIN_CHANGE &&
                Math.abs(sumErrs.yPos) < MIN_CHANGE &&
                Math.abs(sumErrs.angle) < MIN_ANGLE) {
                return false; //Not moving!!!
            }
        }
        return true;
    }
}
