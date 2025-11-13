package org.firstinspires.ftc.teamcode.mechanics;


import static org.firstinspires.ftc.teamcode.stuff.Robot.spind;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tuning.Constants;

public class Spindexer{
    private static double currentX = 1;
    private final static double spindRatio = 93.0 / 22.0;

    public static boolean advance(int angle){
        double target = Constants.CPR * spindRatio / (angle / 360.0) * currentX;
        currentX = currentX + ((angle > 0) ? 1 : -1);
        spind.setTargetPosition((int)target);
        spind.setPower(1);
        spind.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return (Math.abs(spind.getCurrentPosition() - target) < 0.01);
    }
}