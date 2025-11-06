package org.firstinspires.ftc.teamcode.mechanics;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "test",group = "ble")
public class ColorSensorTest extends OpMode {
    public SigmaClass colorSen= new SigmaClass();
    SigmaClass.DetectedColor detectedColor;
    @Override
    public void init(){
        colorSen.init(hardwareMap);
    }
    @Override
    public void loop(){
        detectedColor = colorSen.getColor(telemetry);
        telemetry.addData("color",detectedColor);
    }
}
