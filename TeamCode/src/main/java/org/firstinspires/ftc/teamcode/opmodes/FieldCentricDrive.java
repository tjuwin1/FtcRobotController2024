package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.CurvePoint;
import org.firstinspires.ftc.teamcode.helpers.DataLogger;
import org.firstinspires.ftc.teamcode.helpers.Hardware;
import org.firstinspires.ftc.teamcode.helpers.FieldCentricDriver;

import java.util.ArrayList;

@TeleOp(name="Field Centric Drive", group="Linear OpMode")
public class FieldCentricDrive extends LinearOpMode {
    private Hardware robot;
    private FieldCentricDriver driver;

    private static final double DEAD_ZONE = 0.001;

    private void initializeHardware(){
        robot = new Hardware(hardwareMap,telemetry);
        driver = new FieldCentricDriver(this,telemetry,robot);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void beIdle(){
        robot.localizer.updateReadings();
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initializeHardware();

        while(opModeInInit()){
            beIdle();
        }

        while(opModeIsActive()){
//Drive Robot
            if(Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ||
                    Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ||
                    Math.abs(gamepad1.right_stick_x) > DEAD_ZONE )
                this.driver.followPath();
//Reset IMU
            if(gamepad1.start)
                this.robot.reInitIMU();
        }

        this.driver.stopReleaseAll();
    }
}
