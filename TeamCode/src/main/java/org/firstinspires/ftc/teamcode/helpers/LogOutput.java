package org.firstinspires.ftc.teamcode.helpers;

import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;

public class LogOutput {
    private Telemetry telemetry;
    private static final Paint dumPaint = new Paint();
    private static final float spaceWidth       = 3;

    public LogOutput(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void update(){
        telemetry.update();
    }

    public void Output(String... data){
        String outText = "";
        int titleLength = 275;
        for(int cnt=data.length - 1;cnt>=0;cnt--){
            outText = this.adjustLength(data[cnt],
                        cnt ==0 ? titleLength : 50,
                        cnt ==0 ? true : false) + outText;
            titleLength -= 50;
        }
        telemetry.addLine(outText);
    }

    private static String adjustLength(String input, float length, boolean leftJustified){
        String textToPrint     = input;
        String spaceString     = buildSpaces((int)((length - dumPaint.measureText(textToPrint)) / spaceWidth));

        if(leftJustified){
            textToPrint = textToPrint + spaceString;
        }else {
            textToPrint = spaceString + textToPrint;
        }
        return textToPrint;
    }

    private static String buildSpaces(int repeat){
        if(repeat < 0) return "";
        return String.join("", Collections.nCopies(repeat, " "));
    }

    private static String adjustLength(String input, float length){
        return adjustLength(input,length,false);
    }
}
