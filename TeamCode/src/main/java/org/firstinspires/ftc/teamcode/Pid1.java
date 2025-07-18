package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Pid1 {
    ElapsedTime timer = new ElapsedTime();
    public void loop(){
        poz = motor.getCurrentPosition();
        error = target - poz;
        derivata = (error - lasterror) / timer.seconds();
        sumint = sumint + (error * timer.seconds());
        if(target == 620)
            out = (KP * error) + (KI * sumint) + (KD * derivata);
        else if(target == 0)
            out = ((KP-0.017) * error ) + (KI * sumint) + ((KD - 0.00035) * derivata);
        if(out > 1)
            motor.setPower(1);
        else
            motor.setPower(out);
        lasterror = error;
        timer.reset();
        telemetry.addData("power", out);
        telemetry.addData("error", error);
        telemetry.update();
    }

    DcMotor motor;
    double out;
    double KP = 0.018;
    double KI = 0;
    double KD = 0.00045;
    double target = 0;
    double error;
    double lasterror = 0;
    double poz;
    double derivata = 0;
    double sumint = 0;

    Telemetry telemetry;
    public boolean ok = false;
    public void setOk(int newtarget){
        target = newtarget;
    }

    public Pid1(DcMotor val, Telemetry telemetry) {
        motor = val;
        this.telemetry = telemetry;
        telemetry.update();
    }
}