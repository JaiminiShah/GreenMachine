package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;


@TeleOp(name="GMTeleop2023", group="GreenwoodFTC")  //declares the name of the class and the
// group it is in.



public class GMTeleop2023 extends OpMode{

    //@Disabled
    double  //declares all double variables and their values
            speedVariable = .8,
            armPos = 5,
            armPower = .85,
            intakePower = 1,
            outakePower= -.7,
            armPosIncrement = 4,
            wristPos = .49,
            clawRotateInt = 0.39,
            clawRotate = 0.39,
            boxServoRightClose = 0.65,
            boxServoRightOpen = 0.45,
            boxServoLeftClose = 0.28,
            boxServoLeftOpen = 0.40,
            hangerPos = 0;

    boolean
            clawDone = true,
            intakeRunning = false,
            planeDown = false,
            armMoveDone = true,
            leftOpen = true,
            rightOpen = true,
            clawClosed = true,
            bPressed = false,
            rbPressed = false,
            bImpulse = false,
            rbImpulse = false,
            xPressed = false,
            xImpulse = false;


    final int
            maxArmPos = 1000,
            minArmPos = 0;

    float[] hsvValuesRight = new float[3];
    float[] hsvValuesLeft = new float[3];




    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */
    DcMotor
            frontLeft = null,
            frontRight = null,
            rearLeft = null,
            rearRight = null,
            pixelArm = null,
            intake = null,
            hanger = null;

    Servo
            planeLaunch = null,
            boxServoLeft = null,
            boxServoRight = null,
            pixelWrist = null,
            pixelClawRotate = null;

    HardwareMap hwMap =  null;
    ElapsedTime runTime = new ElapsedTime();

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

    NormalizedColorSensor
            colorSensorLeft = null,
            colorSensorRight = null;

    TouchSensor touchSensor = null;


    @Override
    public void init() { //initialization class to be used at start of tele-op

        // these are our motors and what they are called
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        pixelArm = hardwareMap.dcMotor.get("pixelArm");
        intake = hardwareMap.dcMotor.get("intake");
        hanger = hardwareMap.dcMotor.get("hanger");
        //Servos and their names
        pixelWrist = hardwareMap.servo.get("pixelWrist");
        pixelClawRotate = hardwareMap.servo.get("pixelClawRotate");
        planeLaunch = hardwareMap.servo.get("planeLaunch");
        boxServoLeft = hardwareMap.servo.get("boxServoLeft");
        boxServoRight = hardwareMap.servo.get("boxServoRight");

//Direction?
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        pixelArm.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        hanger.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        //hi
        pixelArm.setPower(0);
        intake.setPower(0);
        hanger.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pixelArm.setTargetPosition((int)armPos);
        pixelArm.setPower(armPower);
        intake.setPower(0);
        hanger.setPower(1);
        //servo set power
        pixelWrist.setPosition(wristPos);
        pixelClawRotate.setPosition(clawRotateInt);
        planeLaunch.setPosition(0.1);
        boxServoRight.setPosition(boxServoRightClose);
        boxServoLeft.setPosition(boxServoLeftClose);



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

        colorSensorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(NormalizedColorSensor.class, "colorSensorRight");
        
        touchSensor = hardwareMap.touchSensor.get("touchSensor");



        //this will send a telemetry message to signify robot waiting;
        telemetry.addLine("AUTOBOTS ROLL OUT");
        telemetry.update();
        runTime.reset();

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

        //Hanger
        if(gamepad1.right_trigger > .5 && hangerPos<5000) hangerPos += 80;
        if(gamepad1.left_trigger > .5&& hangerPos>10) hangerPos -= 40;
        hanger.setTargetPosition((int)hangerPos);

        //Intake
        if(gamepad1.left_bumper){
            intake.setPower(intakePower);
        }
        else if(gamepad1.right_bumper) intake.setPower(outakePower);
        else intake.setPower(0);


        //DriveTrain Speed Controls
        if (gamepad1.dpad_left) speedVariable -= 0.05;
        if (gamepad1.dpad_right) speedVariable += 0.05;
        speedVariable= Range.clip(speedVariable,0,1);

        //==========================================================
        //                        GamePad Two
        //==========================================================

        //D-Pad stuff



        if (gamepad2.dpad_up){
            armPos = 0;
            pixelArm.setPower(0);
            pixelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pixelArm.setPower(armPower);
            armPosIncrement = 1;
        }

        if (gamepad2.dpad_down) armPosIncrement -=0.1;

        if (gamepad2.dpad_left){
            wristPos += .01;
        }
        //Grayson was here 
        if (gamepad2.dpad_right){
            wristPos -= .01;

        }
        pixelWrist.setPosition(wristPos);

        if (gamepad2.right_stick_button){
            armPos = (0);
        }


        if(!xPressed && gamepad2.x) xImpulse = true;
        else xImpulse = false;

        if (gamepad2.x) xPressed = true;
        else if (!gamepad2.x) xPressed = false;

        if(xImpulse) leftOpen = !leftOpen;

        //box servos x and a = open y and b = close a and b = rightboxservo x and y = leftboxservo
        if (leftOpen){ boxServoRight.setPosition(boxServoRightOpen);}
        if (!leftOpen){ boxServoRight.setPosition(boxServoRightClose);}
        if (leftOpen){ boxServoLeft.setPosition(boxServoLeftOpen);leftOpen = true;}
        if (!leftOpen){ boxServoLeft.setPosition(boxServoLeftClose);leftOpen = false;}

        //Arm 
        if (gamepad2.left_trigger > .5 && armPos>minArmPos) armPos-=(armPosIncrement);
        if (gamepad2.right_trigger > .5 && armPos<maxArmPos) armPos+=(armPosIncrement);
        pixelArm.setTargetPosition((int) armPos);

        //plane launcher
        if(!rbPressed && gamepad2.right_bumper) rbImpulse = true;
        else rbImpulse = false;

        if (gamepad2.right_bumper) rbPressed = true;
        else if (!gamepad2.right_bumper) rbPressed = false;

        if(rbImpulse) planeDown = !planeDown;
        if (planeDown){ planeLaunch.setPosition(0.7);}
        if (!planeDown){ planeLaunch.setPosition(0.1);}


        if(gamepad2.right_bumper && planeDown){
            planeLaunch.setPosition(0.7);
            planeDown = false;
        }
        else if(gamepad2.right_bumper && !planeDown){
            planeLaunch.setPosition(0.1);
            planeDown = true;
        }
        if(gamepad2.left_bumper && armMoveDone && pixelArm.getCurrentPosition() < 240){
            armMoveDone = false;
            leftOpen = false;
            rightOpen = false;
            clawRotate = clawRotateInt;
            pixelClawRotate.setPosition(clawRotate);
            armPos = 0;
            pixelArm.setTargetPosition((int)armPos);
            for(int i=0;i<24;i++){
                armPos += 10;
                pixelArm.setTargetPosition((int)armPos);
                clawRotate -= .05;
                pixelClawRotate.setPosition(clawRotate);

                if(armPos > 240) armPos = 240;
            }
            armMoveDone = true;
        }
        else if (gamepad2.left_bumper && armMoveDone && pixelArm.getCurrentPosition() >= 250){
            armPos = 240;
            pixelArm.setTargetPosition((int)armPos);    
        }
        else if(gamepad2.left_bumper && armMoveDone && pixelArm.getCurrentPosition() >= 240){
            armMoveDone = false;
            pixelClawRotate.setPosition(0);
            pixelWrist.setPosition(wristPos-(.0055*48));
            armPos = 240;
            pixelArm.setTargetPosition((int)armPos);
            while(pixelArm.isBusy())telemetry.addLine("hi");
            for(int i=0;i<48;i++){
                armPos -= 5;
                pixelArm.setTargetPosition((int)armPos);
                clawRotate += .006;
                pixelClawRotate.setPosition(clawRotate);

                if(armPos < 0) armPos = 0;
                if(clawRotate > clawRotateInt)clawRotate = clawRotateInt;
            }

            armMoveDone = true;

        }
        if(gamepad2.b && armPos >= 240){
            armPos = 730;
            wristPos = .39;
            clawRotate = .99;
            pixelArm.setTargetPosition(730);
            pixelWrist.setPosition(.39);
            pixelClawRotate.setPosition(.99);
        }

        //Claw Rotate
        if (gamepad2.y) clawRotate +=.02;
        if (gamepad2.a) clawRotate -=.02;
        clawRotate = Range.clip(clawRotate,0,1);
        pixelClawRotate.setPosition(clawRotate);
        if(touchSensor.isPressed()){
            clawRotate += .05;
        }


        //80 yellow 130 green 150 white 210 purple 
        NormalizedRGBA colorRight = colorSensorRight.getNormalizedColors();
        Color.colorToHSV(colorRight.toColor(), hsvValuesRight);

        if(hsvValuesRight[0] > 60 && hsvValuesRight[0] < 100){
            Red1 = false;
            Green1 = false;
            Blue1 = true;
            White1 = true;
        }
        else if(hsvValuesRight[0] > 100 && hsvValuesRight[0] < 140){
            Red1 = true;
            Green1 = false;
            Blue1 = true;
            White1 = true;
        }
        else if(hsvValuesRight[0] > 140 && hsvValuesRight[0] < 180){
            Red1 = false;
            Green1 = false;
            Blue1 = false;
            White1 = false;
        }
        else if(hsvValuesRight[0] > 180 && hsvValuesRight[0] < 250){
            Red1 = false;
            Green1 = true;
            Blue1 = false;
            White1 = true;
        }
        else{
            Red1 = true;
            Green1 = true;
            Blue1 = true;
            White1 = true;
        }


        NormalizedRGBA colorLeft = colorSensorLeft.getNormalizedColors();
        Color.colorToHSV(colorLeft.toColor(), hsvValuesLeft);
        if(hsvValuesLeft[0] > 60 && hsvValuesLeft[0] < 100){
            Red2 = false;
            Green2 = false;
            Blue2 = true;
            White2 = true;
        }
        else if(hsvValuesLeft[0] > 100 && hsvValuesLeft[0] < 140){
            Red2 = true;
            Green2 = false;
            Blue2 = true;
            White2 = true;
        }
        else if(hsvValuesLeft[0] > 140 && hsvValuesLeft[0] < 180){
            Red2 = false;
            Green2 = false;
            Blue2 = false;
            White2 = false;
        }
        else if(hsvValuesLeft[0] > 180 && hsvValuesLeft[0] < 250){
            Red2 = false;
            Green2 = true;
            Blue2 = false;
            White2 = true;
        }
        else{
            Red2 = true;
            Green2 = true;
            Blue2 = true;
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

        //==========================================================//
        //                        Telemetry                           //
        //==========================================================/
        telemetry.addData("Drive Train Speed: " , speedVariable);
        telemetry.addData("Arm Speed(increment)", armPosIncrement);
        telemetry.addData("Target Arm Position",armPos);
        telemetry.addData("Actual Arm Position", pixelArm.getCurrentPosition());
        telemetry.addData("ToggleState", bPressed);
        telemetry.addData("Wrist Position", pixelWrist.getPosition());
        telemetry.addData("Target Rotate Position",clawRotate);
        telemetry.addData("Actual Rotate Position", pixelClawRotate.getPosition());
        telemetry.addData("Hanger Encoder Counts", hanger.getCurrentPosition());
        telemetry.addData("BRMotor", rearRight.getCurrentPosition());
        telemetry.addData("FRMotor", frontRight.getCurrentPosition());
        telemetry.addData("BLMotor", rearLeft.getCurrentPosition());
        telemetry.addData("FLMotor", frontLeft.getCurrentPosition());
        telemetry.addData("HueLeft", "%.3f", hsvValuesLeft[0]);
        telemetry.addData("HueRight", "%.3f", hsvValuesRight[0]);
        telemetry.addData("Touch?",touchSensor.isPressed());

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
        // pixelArm.setTargetPosition(pixelArm.getCurrentPosition());
        intake.setPower(0);
        hanger.setTargetPosition(hanger.getCurrentPosition());
    }
}
//end main

