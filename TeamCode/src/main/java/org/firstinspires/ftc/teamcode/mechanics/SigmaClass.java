package org.firstinspires.ftc.teamcode.mechanics;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SigmaClass {
    NormalizedColorSensor colorSensor;
    public enum DetectedColor{
        PURPLE,
        GREEN,
        UNKNOWN
    }
    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class,"ColorSensorSigma");
        colorSensor.setGain(20);
    }
    public DetectedColor getColor(Telemetry telemetry){
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed=color.red/color.alpha;
        normGreen=color.green/color.alpha;
        normBlue=color.blue/color.alpha;

        telemetry.addData("red",normRed);
        telemetry.addData("blue",normBlue);
        telemetry.addData("green",normGreen);
        /*
        purple = >0.4, >0.1, <0.7
        65
        green = <.3, >0.75, <0.8

         */
        double threshold = 0.2;
        if((normRed<0.14+threshold||normRed>0.14-threshold)&&(normGreen<0.18+threshold||normGreen>0.18-threshold)&&(normBlue<0.2+threshold||normBlue>0.2-threshold))
            return DetectedColor.PURPLE;
        if((normRed<0.13+threshold||normRed>0.13-threshold)&&(normGreen<0.2+threshold||normGreen>0.2-threshold)&&(normBlue<0.22+threshold||normBlue>0.22-threshold))
            return DetectedColor.GREEN;
        return DetectedColor.UNKNOWN;
    }
}
