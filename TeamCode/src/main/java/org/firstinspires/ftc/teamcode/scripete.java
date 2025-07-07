package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class scripete extends OpMode
{
    DcMotor motorscripte;
    @Override
    public void init(){
        motorscripte = hardwareMap.get(DcMotor.class, "motorscripete");
    }
    public void loop(){
        motorscripte.setPower(1
        );
    }
}
