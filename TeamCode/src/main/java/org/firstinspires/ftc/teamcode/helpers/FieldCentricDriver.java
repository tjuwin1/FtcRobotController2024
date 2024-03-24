package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Optional;

public class FieldCentricDriver {
    private Hardware robot;
    private LinearOpMode opMode;
    private Telemetry telemetry;

    public FieldCentricDriver(Hardware robot){
        this.robot = robot;
    }

    public FieldCentricDriver(LinearOpMode opMode,Telemetry telemetry,Hardware robot){
        this.telemetry = telemetry;
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

    public void followPath(){
        if (this.mustContinue()) {
            this.driveInputs();
        }
    }

    private void driveInputs(){
        double powerX = opMode.gamepad1.left_stick_x;
        double powerY = opMode.gamepad1.left_stick_y;
        double powerZ = opMode.gamepad1.right_stick_x;
        double heading = this.getAngle();

        double turnedX = -powerY * Math.sin(heading) + powerX * Math.cos(heading);
        double turnedY = powerY * Math.cos(heading) + powerX * Math.sin(heading);

        double maxPower = Math.abs(turnedX) + Math.abs(turnedY) + Math.abs(powerZ);
        if(maxPower > 1){
            turnedX /= maxPower;
            turnedY /= maxPower;
            powerZ /= maxPower;
        }

        double lfPower = turnedY + turnedX + powerZ;
        double lbPower = turnedY - turnedX + powerZ;
        double rfPower = turnedY - turnedX - powerZ;
        double rbPower = turnedY + turnedX - powerZ;

        robot.leftFrontDrive.setPower(lfPower);
        robot.leftBackDrive.setPower(lbPower);
        robot.rightFrontDrive.setPower(rfPower);
        robot.rightBackDrive.setPower(rbPower);
    }

    private double getAngle(){
        return -(Math.toRadians(robot.imu.getAngularOrientation().firstAngle));
    }
}
