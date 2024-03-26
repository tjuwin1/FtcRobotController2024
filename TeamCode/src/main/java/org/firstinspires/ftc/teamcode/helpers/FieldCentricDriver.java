package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;
import java.util.Optional;

public class FieldCentricDriver {
    private Hardware robot;
    private LinearOpMode opMode;
    private LogOutput telemetry;
    private static final double DEAD_ZONE = 0.001;

    public FieldCentricDriver(Hardware robot){
        this.robot = robot;
    }

    public FieldCentricDriver(LinearOpMode opMode,LogOutput telemetry,Hardware robot){
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
        double powerX =0,powerY =0,powerZ=0;
        if(Math.abs(opMode.gamepad1.left_stick_x) > DEAD_ZONE ||
                Math.abs(opMode.gamepad1.left_stick_y) > DEAD_ZONE ||
                Math.abs(opMode.gamepad1.right_stick_x) > DEAD_ZONE ) {
            powerX = opMode.gamepad1.left_stick_x;
            powerY = opMode.gamepad1.left_stick_y;
            powerZ = opMode.gamepad1.right_stick_x;
        }

        telemetry.Output("X,Y,Z",String.format( "%.3f",powerX),String.format( "%.3f",powerY),String.format( "%.3f",powerZ));

        double heading = this.getAngle();
        double turnedX = -powerY * Math.sin(-heading) + powerX * Math.cos(-heading);
        double turnedY = powerY * Math.cos(-heading) + powerX * Math.sin(-heading);
        telemetry.Output("0°,X₀,Y₀",String.format( "%.3f",Math.toDegrees(heading)),String.format( "%.3f",turnedX),String.format( "%.3f",turnedY));

        double maxPower = Math.abs(turnedX) + Math.abs(turnedY) + Math.abs(powerZ);
        if(maxPower > 1){
            turnedX /= maxPower;
            turnedY /= maxPower;
            powerZ /= maxPower;
        }

        double lfPower = turnedY - turnedX - powerZ;
        double lbPower = turnedY + turnedX - powerZ;
        double rfPower = turnedY + turnedX + powerZ;
        double rbPower = turnedY - turnedX + powerZ;

        robot.leftFrontDrive.setPower(-lfPower);
        robot.leftBackDrive.setPower(-lbPower);
        robot.rightFrontDrive.setPower(-rfPower);
        robot.rightBackDrive.setPower(-rbPower);
    }

    private double getAngle(){
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
}
