package org.firstinspires.ftc.teamcode.helpers;

import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Locale;

public class DataFileLogger {
    private FileOutputStream logFile;

    public DataFileLogger(String fileName, Boolean newFile){
        Date now = Calendar.getInstance().getTime();
        SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss", Locale.US);
        fileName = ( newFile == true ? ( formatter.format(now) + "-" ) : "" ) +
                     fileName + ".log";
        try {
            this.logFile = new FileOutputStream("/storage/emulated/0/FIRST/customLogs/" + fileName);
        }catch(Exception exp){

        }
    }

    private void addLine(String s) {
        if(this.logFile == null){ return; }
        byte[] array = s.getBytes();
        try {
            this.logFile.write(array);
        }catch(Exception exp){
        }
    }

    public void closeLog(){
        try {
            this.logFile.close();
        }catch(Exception exp){
        }
    }

    public void addField(String title,boolean b) {
        addField(title + " : " + ( b ? '1' : '0'));
    }

    public void addField(String title,byte b) {
        addField(title + " : " +  Byte.toString(b));
    }

    public void addField(String title,short s) {
        addField(title + " : " + Short.toString(s));
    }

    public void addField(String title,long l) {
        addField(title + " : " + Long.toString(l));
    }

    public void addField(String title,float f) {
        addField(title + " : " + Float.toString(f));
    }

    public void addField(String title,double d) {
        addField(title + " : " + String.format("%.5f",d));
    }

    public void addField(double... array) {
        String logText="";
        for(int i=0; i<array.length;i++) {
            logText += String.format("%.5f", array[i]);
            if (i != (array.length - 1)){
                logText += ",";
            }
        }
        addLine(logText+"\n");
    }

    public void addField(String... array) {
        String logText="";
        for(int i=0; i<array.length;i++) {
            logText += array[i];
            if (i != (array.length - 1)){
                logText += ",";
            }
        }
        addLine(logText+"\n");
    }
}