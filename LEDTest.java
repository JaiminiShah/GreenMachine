package org.firstinspires.ftc.teamcode;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LightBlinker;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.usb.serial.SerialPort;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.DigitalAllPinsParameters;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.LedColorParameters;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.LedPatternParameters;


@TeleOp(name="LEDTest", group="GreenwoodFTC")  //declares the name of the class and the
// group it is in.



public class LEDTest extends OpMode{


    DigitalChannel LEDWhite1 = null,
            LEDRed1 = null,
            LEDBlue1 = null,
            LEDGreen1 = null,
            LEDWhite2 = null,
            LEDRed2 = null,
            LEDBlue2 = null,
            LEDGreen2 = null;
    Boolean White1 = true,
            Red1 = true,
            Blue1 = true,
            Green1 = true,
            White2 = true,
            Red2 = true,
            Blue2 = true,
            Green2 = true;

    int colorMode = 0;

    HardwareMap hwMap =  null;





    @Override
    public void init() { //initialization class to be used at start of tele-op

        LEDWhite1 = hardwareMap.digitalChannel.get("LEDWhite1");
        LEDWhite1.setMode(DigitalChannel.Mode.OUTPUT);
        LEDRed1 = hardwareMap.digitalChannel.get("LEDRed1");
        LEDRed1.setMode(DigitalChannel.Mode.OUTPUT);
        LEDGreen1 = hardwareMap.digitalChannel.get("LEDGreen1");
        LEDGreen1.setMode(DigitalChannel.Mode.OUTPUT);
        LEDBlue1 = hardwareMap.digitalChannel.get("LEDBlue1");
        LEDBlue1.setMode(DigitalChannel.Mode.OUTPUT);

        LEDWhite2 = hardwareMap.digitalChannel.get("LEDWhite2");
        LEDWhite2.setMode(DigitalChannel.Mode.OUTPUT);
        LEDRed2 = hardwareMap.digitalChannel.get("LEDRed2");
        LEDRed2.setMode(DigitalChannel.Mode.OUTPUT);
        LEDGreen2 = hardwareMap.digitalChannel.get("LEDGreen2");
        LEDGreen2.setMode(DigitalChannel.Mode.OUTPUT);
        LEDBlue2 = hardwareMap.digitalChannel.get("LEDBlue2");
        LEDBlue2.setMode(DigitalChannel.Mode.OUTPUT);





        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");

    }

    /*
     * Code will run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     *this code will run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code will run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //==========================================================
        //                        GamePad One
        //==========================================================

        if(gamepad1.y){
            Red1 = false;
            Red2 = false;
            Green1 = false;
            Green2 = false;
            Blue1 = true;
            Blue2 = true;
            White1 = true;
            White2 = true;


        }
        else if(gamepad1.a){
            Red1 = true;
            Red2 = true;
            Green1 = false;
            Green2 = false;
            Blue1 = true;
            Blue2 = true;
            White1 = true;
            White2 = true;
        }
        else if(gamepad1.x){
            Red1 = false;
            Red2 = false;
            Green1 = true;
            Green2 = true;
            Blue1 = false;
            Blue2 = false;
            White1 = true;
            White2 = true;
        }
        else if(gamepad1.b){
            Red1 = false;
            Red2 = false;
            Green1 = false;
            Green2 = false;
            Blue1 = false;
            Blue2 = false;
            White1 = false;
            White2 = false;
        }
        else{
                Red1 = true;
                Red2 = true;
                Green1 = true;
                Green2 = true;
                Blue1 = true;
                Blue2 = true;
                White1 = true;
                White2 = true;

        }

        LEDWhite1.setState(White1);
        LEDWhite2.setState(White2);
        LEDRed1.setState(Red1);
        LEDRed2.setState(Red2);
        LEDGreen1.setState(Green1);
        LEDGreen2.setState(Green2);
        LEDBlue1.setState(Blue1);
        LEDBlue2.setState(Blue2);


    }

    public void colorSwitch (int color){
        //0 = black, 1 is red, 2 is yellow, 3 is green, 4 is cyan, 5 is blue, 6 is purple, 7 is white
        White1 = true;Red1 = true;Blue1 = true;Green1 = true;White2 = true;Red2 = true;Blue2 = true;Green2 = true;

        if(color == 1 || color ==2||color == 6 || color == 7){
            Red1 = false;
            Red2 = false;
        }
        if(color == 2 || color == 3 || color == 4 || color == 7){
            Green1 = false;
            Green2 = false;
        }
        if(color == 4 || color == 5 || color == 6 || color == 7){
            Blue1 = false;
            Blue2 = false;
        }
        if (color == 7){
            White1 = false;
            White2 = false;
        }
        LEDWhite1.setState(White1);
        LEDWhite2.setState(White2);
        LEDRed1.setState(Red1);
        LEDRed2.setState(Red2);
        LEDGreen1.setState(Green1);
        LEDGreen2.setState(Green2);
        LEDBlue1.setState(Blue1);
        LEDBlue2.setState(Blue2);



    }

    /*
     * Code will run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){

    }
}
//end main
