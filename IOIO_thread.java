package abr.main;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.Uart;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import android.util.Log;
import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.Uart;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;

public class IOIO_thread extends BaseIOIOLooper
{
    //IR sensor
    AnalogInput  irCenter;
    float irCenterReading;
    float dataIRCenter;

    //Servos
    private PwmOutput pwm_pan_output, pwm_tilt_output;
    InputStream in;
    OutputStream out;
    static final int SERVO_PAN_DEFAULT_PWM = 1450, SERVO_TILT_DEFAULT_PWM = 1600, SERVO_MAX_PWM = 2200, SERVO_MIN_PWM = 800;
    static final int DEFAULT_PWM = 1500, MAX_PWM = 2000, MIN_PWM = 1000;
    // < 1500  is Right
    // > 1500 is Left
    private PwmOutput pwm_left_output, pwm_right_output;
    public int pan_value, tilt_value;
    public int pan_direction; // 0 is left, 1 is right;
    public int step = 5;
    public int move = 1;
    public int stop_detected = 0;
    public int stop = 1;
    public int object_detected = 0;
    int flag, flag2, flag3;
    boolean moving = false;
    //Main_activity ma;
    @Override
    public void setup() throws ConnectionLostException
    {
        try {
            //IR Sensor
            irCenter = ioio_.openAnalogInput(43);

            //Servos
            pwm_pan_output = ioio_.openPwmOutput(5, 50);
            pwm_tilt_output = ioio_.openPwmOutput(6, 50);

            //Steer & speed
            pwm_left_output = ioio_.openPwmOutput(3, 50); // S1 to pin 3 controls motor 1 aka left
            pwm_right_output = ioio_.openPwmOutput(4, 50); //S2 to pin 4 controls motor 2 aka right


            //Initial scanning of pan and tilt
            pan_value = SERVO_MIN_PWM;
            tilt_value = 1600;
            pwm_pan_output.setPulseWidth(pan_value);
            pwm_tilt_output.setPulseWidth(tilt_value);

            //set_left(1500);
          //  set_right(1500);

            for(int i = 0; i < (SERVO_MAX_PWM - SERVO_MIN_PWM)/step; i++) {
                pan_value += step;
                pwm_pan_output.setPulseWidth(pan_value);
            }

            for(int i = 0; i < (SERVO_MAX_PWM - SERVO_MIN_PWM)/step; i++) {
                pan_value -= step;
                pan(pan_value );
            }


        }
        catch (ConnectionLostException e){throw e;}
    }

  //  Main_activity ma;

    @Override
    public void loop() throws ConnectionLostException
    {
        ioio_.beginBatch();
        try {
            //pan_value = SERVO_MIN_PWM;
            //tilt_value = 1500;
            //pwm_pan_output.setPulseWidth(pan_value);
            pwm_tilt_output.setPulseWidth(1600);
            if(pan_direction == 1){
                if(pan_value + step < SERVO_MAX_PWM - 350) { // move right
                    pan_value += step;
                    pan(pan_value);
                }
                else { // change direction to left
                    pan_direction = 0;
                }
            }
            else if(pan_direction == 0){
                if(pan_value + step > SERVO_MIN_PWM + 350) {//move left
                    pan_value -= step;
                    pan(pan_value);
                }
                else{ // change direction to right
                    pan_direction = 1;
                }
            }

            //IR Sensor
            dataIRCenter = irCenter.read();
       //     System.out.println("MOVING object dataIRCenter" + dataIRCenter);
            if((flag3 == 0)&&(flag2 == 0)&&(flag == 0)) {
              //  System.out.println("MOVING object dataIRCenter" + dataIRCenter);
                if (dataIRCenter > 0.4) {
                    set_left(1250);
                    set_right(1250);
                    flag = 33;
                    move = 0;
                    object_detected = 1;
                    System.out.println("object detected!!!!!!!!!!!!!!");

                }
            }
            //irCenterReading = irCenter.getVoltage();

           //ADD BACK IN for moving on start and then just testing ir AND COMMENT OUT COLOR DETECTION CODE
            //Default movement
           /* if(move == 1 && object_detected != 1){
                    move(1700);
                    moving = true;
            }

            else*/ if(stop_detected == 1 && stop == 1){
                move = 0;
                stop = 0;
                stop_detected = 0;
                //stop for ten secs
                Thread.sleep( 10*1000, 0);
                move = 1;
            }

            else if(object_detected == 1){
                if(flag > 0){
                    flag--;
                    if(flag ==0){
                        set_left(1700);
                        set_right(1580);
                        flag2 = 55;
                    }
                }

                if(flag2 > 0){
                    flag2--;
                    if(flag2 ==0){
                        set_left(1580);
                        set_right(1750);
                        flag3 = 60;
                    }
                }

                if(flag3 > 0){
                    flag3--;
                    if(flag3 ==0){
                        set_left(1500);
                        set_right(1500);
                        move = 1;
                        object_detected = 0;
                    }
                }
                //move = 1;
                //object_detected = 0;
            }
            Thread.sleep(10);
        } catch (InterruptedException e) {
            ioio_.disconnect();
        } finally {
            ioio_.endBatch();
        }
    }


    public float getIrCenterReading() {
        return dataIRCenter/*irCenterReading*/;
    }


    public synchronized void stay(){
        set_left(1500);
        set_right(1500);
    }
    public synchronized void move(int value){
        set_left(value);
        set_right(value);
    }
    public synchronized void turn_left(int value){
        set_left(value);
        set_right(value+200);
    }
    public synchronized void turn_right(int value){
        set_left(value+200);
        set_right(value);
    }

    public synchronized void pan(int value)
    {
        try {
            if(value > SERVO_MAX_PWM)
                pwm_pan_output.setPulseWidth(SERVO_PAN_DEFAULT_PWM);
            else if(value < SERVO_MIN_PWM)
                pwm_pan_output.setPulseWidth(SERVO_PAN_DEFAULT_PWM);
            else
                pwm_pan_output.setPulseWidth(value);
        } catch (ConnectionLostException e) {
            ioio_.disconnect();
        }

    }

    public synchronized void tilt(int value)
    {
        try {
            if(value > SERVO_MAX_PWM)
                pwm_tilt_output.setPulseWidth(SERVO_TILT_DEFAULT_PWM);
            else if(value < SERVO_MIN_PWM)
                pwm_tilt_output.setPulseWidth(SERVO_TILT_DEFAULT_PWM);
            else
                pwm_tilt_output.setPulseWidth(value);
        } catch (ConnectionLostException e) {
            ioio_.disconnect();
        }
    }

    public synchronized void set_left(int value) {
        try {
            if(value > MAX_PWM)
                pwm_left_output.setPulseWidth(MAX_PWM);
            else if(value < MIN_PWM)
                pwm_left_output.setPulseWidth(MIN_PWM);
            else {
                if (pwm_left_output != null) {
                    pwm_left_output.setPulseWidth(value);
                }
            }
        } catch (ConnectionLostException e) {
            ioio_.disconnect();
        }
    }

    public synchronized void set_right(int value){
        try {
            if(value > MAX_PWM)
                pwm_right_output.setPulseWidth(MAX_PWM);
            else if(value < MIN_PWM)
                pwm_right_output.setPulseWidth(MIN_PWM);
            else {
                if (pwm_right_output != null) {
                    pwm_right_output.setPulseWidth(value);
                }
            }
        } catch (ConnectionLostException e) {
            ioio_.disconnect();
        }
    }
}