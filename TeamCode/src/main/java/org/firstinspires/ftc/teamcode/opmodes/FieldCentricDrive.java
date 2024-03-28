package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.CurvePoint;
import org.firstinspires.ftc.teamcode.helpers.DataFileLogger;
import org.firstinspires.ftc.teamcode.helpers.Hardware;
import org.firstinspires.ftc.teamcode.helpers.FieldCentricDriver;
import org.firstinspires.ftc.teamcode.helpers.LogOutput;

import java.util.ArrayList;

@TeleOp(name="FieldCentric Drive", group="Linear OpMode")
public class FieldCentricDrive extends LinearOpMode {
    private Hardware robot;
    private FieldCentricDriver driver;
    private LogOutput logOutput;

    private void initializeHardware(){
        logOutput = new LogOutput(telemetry);
        robot = new Hardware(hardwareMap,logOutput);
        driver = new FieldCentricDriver(this,logOutput,robot);

        logOutput.Output("Status", "Initialized");
    }

    private void waitForInit(){
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initializeHardware();
        telemetry.update();
        while(opModeInInit()){
            waitForInit();
            telemetry.update();
        }

        while(opModeIsActive()){
//Drive Robot
            this.driver.followPath();

//Reset IMU
            if(gamepad1.start)
                this.robot.reInitIMU();
            telemetry.update();
        }

        this.driver.stopReleaseAll();
    }
}
