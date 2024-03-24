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

    public static final double TURN_AT_DISTANCE = 10;

    //Values for Proportional in PID tuning
    private static final double kPxy=0.12, kP0=1;
    private static final double kDxy=0.012,kD0=0.045;
    private static final double kIxy=0.0012, kI0=0.0045;

    private final double MIN_ANGLE = Math.toRadians(1);
    private final double MIN_DISTANCE = 0.5;
    private final double MIN_CHANGE = 0.05;

    private LinearOpMode opMode;
    private Telemetry telemetry;
    private DataLogger logger;
    private ElapsedTime runtime = new ElapsedTime();

    private ArrayList<CurvePoint> prevDeltas = new ArrayList<CurvePoint>();

    public PurePursuitDriver(DataLogger logger, Hardware robot){
        this.logger = logger;
        this.robot = robot;
    }

    public PurePursuitDriver(LinearOpMode opMode,Telemetry telemetry, DataLogger logger, Hardware robot){
        this.telemetry = telemetry;
        this.logger = logger;
        this.opMode = opMode;
        this.robot = robot;
    }

    private boolean mustContinue(){
        return this.opMode.opModeIsActive();
    }

    public void stopReleaseAll(){
        robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }

    public void followPath(ArrayList<CurvePoint> allPoints){
        if(!this.mustContinue()) {
            return;
        }

        Boolean firstRun = true;
        CurvePoint prevPoint = new CurvePoint(0, 0, 0);
        CurvePoint deltaError = new CurvePoint(0, 0, 0);
        CurvePoint integralPoint = new CurvePoint(0, 0, 0);

        prevDeltas.clear();
        runtime.reset();

        logger.addField(new String[]{
                "cycleTime","xPos", "yPos", "angle","tgtX","tgtY","tgta",
                "distanceToTarget", "absoluteAngleToTarget",
                "relativeAngleToPoint", "relativeTurnAngle", "xyTotal", "xPower", "yPower",
                "dxPos", "dyPos", "dangle", "xPIDPower", "yPIDPower",
                "aPIDPower", "lfPower", "lbPower", "rfPower", "rbPower", "\n"});

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

            if (firstRun == false) {
                deltaError = CurvePoint.subtract(prevPoint, new CurvePoint(xPower, yPower, relativeTurnAngle));
                integralPoint = CurvePoint.add(integralPoint, CurvePoint.multiply(deltaError, cycleTime));
                addDelta(deltaError);
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

            robot.leftFrontDrive.setPower(lfPower);
            robot.leftBackDrive.setPower(lbPower);
            robot.rightFrontDrive.setPower(rfPower);
            robot.rightBackDrive.setPower(rbPower);

            logger.addField(new double[]{cycleTime,
                    robot.localizer.robotPose.xPos, robot.localizer.robotPose.yPos, Math.toDegrees(robot.localizer.robotPose.angle),
                    point.xPos,point.yPos,point.angle,
                    distanceToTarget, Math.toDegrees(absoluteAngleToTarget),
                    Math.toDegrees(relativeAngleToPoint), Math.toDegrees(relativeTurnAngle), xyTotal, xPower, yPower,
                    deltaError.xPos, deltaError.yPos, Math.toDegrees(deltaError.angle),
                    xPIDPower, yPIDPower, aPIDPower, lfPower, lbPower, rfPower, rbPower});

            telemetry.update();
            if (Math.abs(relativeTurnAngle) <= MIN_ANGLE && distanceToTarget <= MIN_DISTANCE) {
                logger.addField("\n-Reached--------------------------\n");
                return;
            } else if (!amImoving()) {
                logger.addField("\n-Stuck--------------------------\n");
                return;
            }
            logger.addField("\n");
            firstRun = false;
        }
    }

    private void addDelta(CurvePoint deltaPoint){
        prevDeltas.add(deltaPoint);
//Capture a maximum of 10 deltas
        while (prevDeltas.size() > 10){
            prevDeltas.remove(0);
        }
    }

    private boolean amImoving(){
        if(prevDeltas.size() == 10) {
            CurvePoint sumDelta = new CurvePoint(0, 0, 0);
            for (int cnt = 0; cnt < prevDeltas.size(); cnt++) {
                sumDelta = CurvePoint.add(sumDelta, prevDeltas.get(cnt));
            }
            if (Math.abs(sumDelta.xPos) < MIN_CHANGE && Math.abs(sumDelta.yPos) < MIN_CHANGE && Math.abs(sumDelta.angle) < MIN_ANGLE) {
                return false; //Not moving!!!
            }
        }
        return true;
    }
}
