package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous
public class autono extends  OpMode{
    double poz = 0;
    double dist = 100;
    double error, lasterror, Serror;
    double P = 0.01, I = 0.001, D = 0.01;
    double power;
    public void init(){

    }
    @Override
    public void loop() {
        error = dist - poz;
        Serror += error;
        power = P * error + I * Serror + D * (error - lasterror);
        lasterror = error;

        DcMotor motorsst, motorsdr, motorfst, motorfdr;
        motorfst = hardwareMap.get(DcMotor.class, "mfs");
        motorfdr = hardwareMap.get(DcMotor.class, "mfd");
        motorsst = hardwareMap.get(DcMotor.class, "mss");
        motorsdr = hardwareMap.get(DcMotor.class, "msd");

        motorfdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorfst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorfdr.setDirection(DcMotorSimple.Direction.REVERSE);
        motorsdr.setDirection(DcMotorSimple.Direction.REVERSE);
        motorfdr.setPower(power);
        motorfst.setPower(power);
        motorsst.setPower(power);
        motorsdr.setPower(power);
    }
}
