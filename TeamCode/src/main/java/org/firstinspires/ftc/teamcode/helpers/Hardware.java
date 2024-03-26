package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Hardware {
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public Odometry localizer;

    protected DcMotor rightPod = null;
    protected DcMotor leftPod = null;
    protected DcMotor backPod = null;
    protected BNO055IMU imu;

    private HardwareMap hardwareMap;
    protected LogOutput logOutput;

    private DcMotor initializeMotor(String motorName, DcMotorSimple.Direction direction){
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(direction);
        return motor;
    }

    public Hardware(HardwareMap hardwareMap, LogOutput logOutput){
        this.hardwareMap = hardwareMap;
        this.logOutput = logOutput;

        leftPod = leftFrontDrive  = initializeMotor("frontLeft",DcMotor.Direction.REVERSE);
        backPod = leftBackDrive  = initializeMotor("backLeft",DcMotor.Direction.REVERSE);
        rightPod = rightFrontDrive = initializeMotor("frontRight",DcMotor.Direction.FORWARD);
        rightBackDrive = initializeMotor( "backRight",DcMotor.Direction.FORWARD);
        localizer = new Odometry(this,logOutput);

        this.reInitIMU();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void reInitIMU(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters IMUParameters;
        IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.mode = BNO055IMU.SensorMode.IMU;
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(IMUParameters);
    }
}
