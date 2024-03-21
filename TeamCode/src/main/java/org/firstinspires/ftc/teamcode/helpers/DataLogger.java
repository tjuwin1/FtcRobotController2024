package org.firstinspires.ftc.teamcode.helpers;

import android.os.Environment;

import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;

public class DataLogger {
    private File logFile;
    private String logText ="";

    public DataLogger(String fileName, Boolean newFile) {
        Date now = Calendar.getInstance().getTime();
        SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss", Locale.US);
        fileName = ( newFile == true ? ( formatter.format(now) + "-" ) : "" ) +
                     fileName + ".log";
        this.logFile = new File("/storage/emulated/0/FIRST/customLogs/" + fileName);
    }

    public void addField(String s) {
        logText += s;
    }

    public void closeLog(){
        ReadWriteFile.writeFile(this.logFile, this.logText);
    }

    public void addField(char c) { addField(String.valueOf(c)); }

    public void addField(boolean b) {
        addField(b ? '1' : '0');
    }

    public void addField(byte b) {
        addField(Byte.toString(b));
    }

    public void addField(short s) {
        addField(Short.toString(s));
    }

    public void addField(long l) {
        addField(Long.toString(l));
    }

    public void addField(float f) {
        addField(Float.toString(f));
    }

    public void addField(double d) {
        addField(String.format("%.5f",d));
    }

    public void addField(double[] array) {
        for(int i=0; i<array.length;i++) {
            addField(String.format("%.5f", array[i]));
            if (i != (array.length - 1)){
                addField(",");
            }
        }
    }

    public void addField(String[] array) {
        for(int i=0; i<array.length;i++) {
            addField(array[i]);
            if (i != (array.length - 1)){
                addField(",");
            }
        }
    }
}