package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
import com.qualcomm.robotcore.hardware.usb.serial.SerialPort;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.AnalogCommands;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.LedCommands;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.LedColorParameters;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.LedPatternParameters;


@TeleOp(name="WALK", group="GreenwoodFTC")  //declares the name of the class and the
// group it is in.



public class WALK extends OpMode{

    //@Disabled
    double  //declares all double variables and their values
            speedVariable = .6;

    //boolean
        
        
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



    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */
    DcMotor
            frontLeft = null,
            frontRight = null,
            rearLeft = null,
            rearRight = null;


    HardwareMap hwMap =  null;
    





    @Override
    public void init() { //initialization class to be used at start of tele-op

        // these are our motors and what they are called
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        




//Direction?
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        
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

        //Controls Drive Train

        float FLspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        float BLspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        float FRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x;
        float BRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x;

        rearLeft.setPower(Range.clip((-BLspeed*speedVariable),-1,1));
        rearRight.setPower(Range.clip((BRspeed*speedVariable),-1,1));
        frontLeft.setPower(Range.clip((FLspeed*speedVariable),-1,1));
        frontRight.setPower(Range.clip((-FRspeed*speedVariable),-1,1));



        //==========================================================
        //                        GamePad Two
        //==========================================================


        if (gamepad1.dpad_up) speedVariable += 0.0001;
        if (gamepad1.dpad_down) speedVariable -= 0.0001;
        speedVariable = Range.clip(speedVariable,0,1);

        //==========================================================//
        //                        Telemetry                           //
        //==========================================================/
        telemetry.addData("speedVariable speed" , speedVariable);
        telemetry.addData("BRMotor", rearRight.getPower());
        telemetry.addData("FRMotor", frontRight.getPower());
        telemetry.addData("BLMotor", rearLeft.getPower());
        telemetry.addData("FLMotor", frontLeft.getPower());

        
            Red1 = false;
            Red2 = true;
            Green1 = false;
            Green2 = false;
            Blue1 = true;
            Blue2 = true;
            White1 = true;
            White2 = true;

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
        // Sets all motors to zero power except Arms to keep pos
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);

    }
}
//end main
