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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helpers.CurvePoint;
import org.firstinspires.ftc.teamcode.helpers.DataLogger;
import org.firstinspires.ftc.teamcode.helpers.Driver;
import org.firstinspires.ftc.teamcode.helpers.Hardware;

import java.util.ArrayList;

@TeleOp(name="Straight Line With Odometry", group="Linear OpMode")
public class StraightLineWithOdometry extends LinearOpMode {

    private DataLogger logger;
    private Hardware robot;
    private Driver driver;

    private void initializeHardware(){
        robot = new Hardware(hardwareMap,telemetry);

        logger = new DataLogger("StraightLineWithOdometry",false);

        driver = new Driver(this,telemetry,logger,robot);

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

        if(opModeIsActive()){
            ArrayList<CurvePoint> newPath = new ArrayList<CurvePoint>();
            newPath.add(new CurvePoint(20,0,Math.toRadians(90)));
            newPath.add(new CurvePoint(30,40,Math.toRadians(180)));
            this.driver.followPath(newPath);
        }

        this.driver.stopReleaseAll();

        logger.closeLog();
    }

}
