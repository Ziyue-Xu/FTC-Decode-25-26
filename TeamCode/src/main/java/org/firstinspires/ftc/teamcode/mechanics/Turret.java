package org.firstinspires.ftc.teamcode.mechanics;


import static org.firstinspires.ftc.teamcode.stuff.Robot.turn;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tuning.Constants;

public class Turret{
    private static double currentAngle = 0;
    private final static int turretRatio = 7;

    public static boolean pointTurret(double desiredAngle){
        double target = (desiredAngle / (2 * Math.PI)) * Constants.CPR * turretRatio;
        turn.setTargetPosition((int)target);
        turn.setPower(1);
        turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return (Math.abs(currentAngle - desiredAngle) < 0.01);
    }
}