package org.firstinspires.ftc.teamcode.helpers;
import android.graphics.Paint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Odometry{
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private static final double trackWidth = 9.605171684632;
    private static final double backOffset = 5.444241253624781;
    private static final double inPerTick = 4.8 * Math.PI / 2000 / 2.54;

    private int leftOdoOff=0, rightOdoOff=0, backOdoOff=0;
    private int prevRightOdo, prevLeftOdo, prefBackOdo;

    public CurvePoint robotPose = new CurvePoint(0,0,0);
    private LogOutput telemetry;

    public Odometry(Hardware hardwareMap,LogOutput telemetry){
        this.telemetry = telemetry;
        leftFrontDrive  = hardwareMap.leftFrontDrive;
        leftBackDrive  = hardwareMap.leftBackDrive;
        rightFrontDrive = hardwareMap.rightFrontDrive;
        rightBackDrive = hardwareMap.rightBackDrive;
        this.prevRightOdo = this.rightOdoOff = - rightFrontDrive.getCurrentPosition();
        this.prefBackOdo = this.backOdoOff = - leftBackDrive.getCurrentPosition();
        this.prevLeftOdo = this.leftOdoOff = - leftFrontDrive.getCurrentPosition();
    }

    public void setRobotPos(CurvePoint Pose){
        this.robotPose.xPos = Pose.xPos;
        this.robotPose.yPos = Pose.yPos;
        this.robotPose.angle = Pose.angle;
    }

    public void updateReadings(){
        double r0, r1;
        int rightOdo = - rightFrontDrive.getCurrentPosition() - this.rightOdoOff;
        int backOdo = - leftBackDrive.getCurrentPosition() - this.backOdoOff;
        int leftOdo = - leftFrontDrive.getCurrentPosition() - this.leftOdoOff;

        if (rightOdo != this.prevRightOdo || backOdo != this.prefBackOdo || leftOdo != this.prevLeftOdo) {
            double deltaLeft = leftOdo - this.prevLeftOdo;
            double deltaRight = rightOdo - this.prevRightOdo;
            double deltaBack = backOdo - this.prefBackOdo;

            double deltaFwd = (deltaLeft + deltaRight) * inPerTick / 2;
            double delta0 = (deltaLeft - deltaRight) * inPerTick / trackWidth;
            double deltaStrafe = deltaBack * inPerTick - backOffset * delta0;

            if (delta0 == 0) {
                delta0 = 1E-7;
            }
            r0 = deltaFwd / delta0;
            r1 = deltaStrafe / delta0;

            double relX = r0 * Math.sin(delta0) - r1 * (1 - Math.cos(delta0));
            double relY = r1 * Math.sin(delta0) + r0 * (1 - Math.cos(delta0));

            this.robotPose.angle -= delta0;
            this.robotPose.angle = MathFunctions.AngleWrap(this.robotPose.angle);
            this.robotPose.xPos += relX * Math.cos(this.robotPose.angle) - relY * Math.sin(this.robotPose.angle);
            this.robotPose.yPos += relY * Math.cos(this.robotPose.angle) + relX * Math.sin(this.robotPose.angle);

            this.prevRightOdo = rightOdo;
            this.prefBackOdo = backOdo;
            this.prevLeftOdo = leftOdo;
        }

        this.printPosition(rightOdo,backOdo,leftOdo);
    }

    private void printPosition(int rightOdo,int backOdo,int leftOdo){
        telemetry.Output("Odo Pos (R,B,L):",String.valueOf(rightOdo),String.valueOf(backOdo),String.valueOf(leftOdo));
        telemetry.Output("Robot Pos (X,Y,θ°):",String.format( "%.3f",this.robotPose.xPos),String.format( "%.3f",this.robotPose.yPos),String.format( "%.3f",Math.toDegrees(this.robotPose.angle)));
    }
}