/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import android.provider.ContactsContract;
import android.util.Log;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.helpers.CurvePoint;
import org.firstinspires.ftc.teamcode.helpers.DataLogger;
import org.firstinspires.ftc.teamcode.helpers.MathFunctions;
import org.firstinspires.ftc.teamcode.helpers.Odometry;

@TeleOp(name="Straight Line With Odometry", group="Linear OpMode")
public class StraightLineWithOdometry extends LinearOpMode {
    private static final double kP0=1, kPx=1, kPy=1; //Values for Proportional in PID tuning
    private static final double speed=0.5;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private BNO055IMU imu;
    private Odometry myPosition;
    private DataLogger logger;

    private void initializeHardware(){
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters IMUParameters;
        IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.mode = BNO055IMU.SensorMode.IMU;
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(IMUParameters);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        myPosition = new Odometry(hardwareMap,telemetry);
        logger = new DataLogger("StraightLineWithOdometry",false);

        logger.addField(new String[] {"cycleTime","robotX","robotY","robot0","distanceToTarget","absoluteAngleToTarget","relativeAngleToPoint",
                "relativeTurnAngle","xyTotal","xPower","yPower","aPower","lfPower","lbPower","rfPower","rbPower","\n"});

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    private void beIdle(){
        myPosition.updateReadings();
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initializeHardware();

        // Wait for the game to start (driver presses PLAY)
        while(opModeInInit()){
            beIdle();
        }

        if(opModeIsActive()){
            runtime.reset();
            while (opModeIsActive() && this.moveTo(new CurvePoint(15,15,Math.toRadians(45)))){}
            logger.addField("\n-------------------------------\n");
            while (opModeIsActive() && this.moveTo(new CurvePoint(-15,15,Math.toRadians(135)))){}
            logger.addField("\n-------------------------------\n");
            while (opModeIsActive() && this.moveTo(new CurvePoint(-15,-15,Math.toRadians(225)))){}
            logger.addField("\n-------------------------------\n");
            while (opModeIsActive() && this.moveTo(new CurvePoint(15,-15,Math.toRadians(-45)))){}
            logger.addField("\n-------------------------------\n");
            while (opModeIsActive() && this.moveTo(new CurvePoint(0,0,Math.toRadians(0)))){}
        }

        this.stopReleaseAll();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            beIdle();
        }
        logger.closeLog();
    }

    private boolean moveTo(CurvePoint point){
        myPosition.updateReadings();
        double cycleTime = runtime.milliseconds();
        runtime.reset();//Reset timer to get cycle time for each loop
        //Find the maximum allowed power change in a second
        double lfPowerMax = Range.clip(Math.abs(leftFrontDrive.getPower()) + cycleTime / 100, -1,1);
        double lbPowerMax = Range.clip(Math.abs(leftBackDrive.getPower()) + cycleTime / 100, -1,1);
        double rfPowerMax = Range.clip(Math.abs(rightFrontDrive.getPower()) + cycleTime / 100, -1,1);
        double rbPowerMax = Range.clip(Math.abs(rightBackDrive.getPower()) + cycleTime / 100, -1,1);

        double distanceToTarget = Math.hypot(point.xPos - myPosition.robotPose.xPos, point.yPos - myPosition.robotPose.yPos);
        double absoluteAngleToTarget = Math.atan2(point.yPos - myPosition.robotPose.yPos, point.xPos - myPosition.robotPose.xPos);
        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - myPosition.robotPose.angle);
        double xPower = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double yPower = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double relativeTurnAngle = Range.clip(MathFunctions.AngleWrap(point.angle - myPosition.robotPose.angle),-Math.PI/2,Math.PI/2);
        double aPower = Math.sin(relativeTurnAngle);

        xPower *= kPx;
        yPower *= kPy;
        aPower *= kP0;

        double xyTotal = Math.abs(xPower) + Math.abs(yPower) + Math.abs(aPower);
        if (xyTotal > 1){
            xPower /= xyTotal;
            yPower /= xyTotal;
            aPower /= xyTotal;
        }

        xPower *= speed;
        yPower *= speed;

        //Limit the acceleration of the motors
        double lfPower = Range.clip(xPower - yPower - aPower,-lfPowerMax,lfPowerMax); //(y + x + rx)
        double lbPower = Range.clip(xPower + yPower - aPower,-lbPowerMax,lbPowerMax); //(y - x + rx)
        double rfPower = Range.clip(xPower + yPower + aPower,-rfPowerMax,rfPowerMax); //(y - x - rx)
        double rbPower = Range.clip(xPower - yPower + aPower,-rbPowerMax,rbPowerMax); //(y + x - rx)

        leftFrontDrive.setPower(lfPower);
        leftBackDrive.setPower(lbPower);
        rightFrontDrive.setPower(rfPower);
        rightBackDrive.setPower(rbPower);

        logger.addField(new double[] {cycleTime,
                myPosition.robotPose.xPos,myPosition.robotPose.yPos,Math.toDegrees(myPosition.robotPose.angle),
                distanceToTarget,Math.toDegrees(absoluteAngleToTarget),
                Math.toDegrees(relativeAngleToPoint),Math.toDegrees(relativeTurnAngle),xyTotal,xPower,yPower,aPower,lfPower,lbPower,rfPower,rbPower});

        telemetry.update();
        if(Math.abs(relativeTurnAngle) <= Math.toRadians(2) && distanceToTarget <= 1){
            return false;
        }
        logger.addField("\n");
        return true;
    }

    private void stopReleaseAll(){
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private double getAngle(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
