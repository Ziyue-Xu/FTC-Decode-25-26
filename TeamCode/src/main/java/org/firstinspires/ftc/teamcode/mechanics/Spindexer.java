package org.firstinspires.ftc.teamcode.mechanics;


import static org.firstinspires.ftc.teamcode.stuff.Robot.spind;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.tuning.Constants;

public class Spindexer{
    private final static double spindRatio = 1;

    public static boolean advance(int angle){
        double target = Constants.CPR * spindRatio / (angle / 360.0);
        spind.setTargetPosition((int)target);
        spind.setPower(1);
        spind.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (Math.abs(spind.getCurrentPosition() - target) <= 10)
            spind.setPower(0);
        return (Math.abs(spind.getCurrentPosition() - target) < 0.01);
    }
}