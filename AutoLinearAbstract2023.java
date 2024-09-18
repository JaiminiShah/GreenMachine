/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public abstract class AutoLinearAbstract2023 extends LinearOpMode {



    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * Common autonomous opmode members
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * Automated objects, timers, variables, constants
     * ------------------------------------------------------- */

    // OBJECTS
    mecanumDrive
            driveTrain;

    // Where you would declare any additional motors
    DcMotor
            pixelArm = null,
            intake = null,
            hanger = null;

    //Where you would declare any servos for the robot
    DeviceTargetServo
            pixelWrist = null,
            pixelClawRotate = null,
            planeLaunch = null,
            boxServoLeft = null,
            boxServoRight = null;


    //declares timers that we use
    ElapsedTime
            generalTimer = new ElapsedTime(), // General/multipurpose timer
            autoTimer = new ElapsedTime();    // Autonomous timer

    IMU imu;

    TouchSensor touchSensor = null;
    ColorSensor colorSensorLeft = null, colorSensorRight = null;
    
    DigitalChannel LEDWhite1 = null,
            LEDRed1 = null,
            LEDBlue1 = null,
            LEDGreen1 = null,
            LEDWhite2 = null,
            LEDRed2 = null,
            LEDBlue2 = null,
            LEDGreen2 = null;

    public HuskyLens huskyLens;

    //  -------------------------Variables----------------------------------------------
    //any words we might need to store
    public String
            team = "",
            initialPos = "";

    //True-false variables
    boolean
            safeStop,
            RunAutoInput;
    // any whole number variables
    final int READ_PERIOD = 1;


    // any decimal values
    double
            robotHeading  = 0,
            headingOffset = 0,
            headingError  = 0,
            turnTarget = 0,
            targetHeading = 0,
            objectLocation = 1,
            wristPos = .49,
            clawRotateInt = 0.39,
            clawRotate = 0.69,
            boxServoRightClose = 0.65,
            boxServoRightOpen = 0.45,
            boxServoLeftClose = 0.28,
            boxServoLeftOpen = 0.40,
            hangerPos = 0;


    // constants: declared here and can never be changed further in the program
    // however, changing one of these numbers will change it for all(speeds)
    final double
            MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES = .25,
            DRIVE_TRAIN_STRAIGHT_SPEED = 0.9,
            DRIVE_TRAIN_DEFAULT_SPEED = 0.7,
            HEADING_THRESHOLD       = 0.25,
            P_TURN_GAIN            = 0.1;     // Larger is more responsive, but also less stable

    final boolean
            FORWARD = false,
            REVERSE = true;



    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: runOpMode (Overridden Linear OpMode)
     * Purpose: Establish and initialize automated objects, and signal initialization completion
     * ------------------------------------------------------- */
    @Override
    public void runOpMode() {

        safeStop = false; //Used for stopping robot
        /* INITIALIZE ROBOT - ESTABLISH ROBOT OBJECTS */

        /* Drive Train constructor: hardwareMap, left motor name, left motor direction, right motor name, right motor direction,
                                    encoder counts per output shaft revolution, gear ratio, wheel radius */
        driveTrain = new mecanumDrive(hardwareMap, "frontLeft",REVERSE, "frontRight", FORWARD, "rearLeft", REVERSE, "rearRight", FORWARD, 538, 1, 2.0);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */


        pixelArm = hardwareMap.dcMotor.get("pixelArm");
        pixelArm.setDirection(DcMotor.Direction.REVERSE);
        pixelArm.setPower(0);
        pixelArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pixelArm.setPower(0.7);
        pixelArm.setTargetPosition(0);


        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hanger = hardwareMap.dcMotor.get("hanger");
        hanger.setDirection(DcMotor.Direction.REVERSE);
        hanger.setPower(0);
        hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hanger.setPower(1);
        hanger.setTargetPosition(0);


        pixelWrist = new DeviceTargetServo(hardwareMap,"pixelWrist",wristPos);
        pixelClawRotate = new DeviceTargetServo(hardwareMap,"pixelClawRotate",clawRotateInt);
        planeLaunch = new DeviceTargetServo(hardwareMap,"planeLaunch",.1);
        boxServoLeft = new DeviceTargetServo(hardwareMap,"boxServoLeft",boxServoLeftClose);
        boxServoRight = new DeviceTargetServo(hardwareMap,"boxServoRight",boxServoRightClose);


        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        double xRotation = 90;  // enter the desired X rotation angle here.
        double yRotation = -90;  // enter the desired Y rotation angle here.
        double zRotation = 0;  // enter the desired Z rotation angle here.
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        orientation.getRoll(AngleUnit.DEGREES);


        rateLimit.expire();
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        if (!huskyLens.knock()) telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorSensorRight = hardwareMap.colorSensor.get("colorSensorRight");
        
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



        // Target-Servo constructor: hardwareMap, servo name, initial servo position

        // Notify drive station that robot objects are being initialized
        telemetry.addLine("Wait - Initializing Robot Objects");
        telemetry.update();

         /* Reset encoders and place motors into the 'Run-to-Position' mode
            Note: The initialization calls in the following methods could not be performed in the respective
           object constructors */
        driveTrain.resetEncoders();


        /* Lock drive train at current position */
        driveTrain.front.motorLeft.goToAbsoluteDistance(driveTrain.front.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.front.motorRight.goToAbsoluteDistance(driveTrain.front.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.rear.motorLeft.goToAbsoluteDistance(driveTrain.rear.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.rear.motorRight.goToAbsoluteDistance(driveTrain.rear.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);


        if (RunAutoInput) {
            telemetry.addLine("Blue or Red ? (X or B)");
            telemetry.update();
            while (team == "") {
                if (gamepad1.x) {
                    team = "Blue";
                }
                if (gamepad1.b) {
                    team = "Red";
                }
                if (isStarted())break;
            }
            sleep(500);
            if (team == "Blue") {
                telemetry.addLine("BlueLeft(A4) or BlueRight(A2)? (X or B)");
            }
            if (team == "Red") {
                telemetry.addLine("RedLeft(F2) or RedRight(F4)? (X or B)");
            }
            telemetry.update();

            while (initialPos == "") {
                if (gamepad1.x && team == "Red") {
                    initialPos = "F2";
                }
                if (gamepad1.x && team == "Blue") {
                    initialPos = "A4";
                }
                if (gamepad1.b && team == "Red") {
                    initialPos = "F4";
                }
                if (gamepad1.b && team == "Blue") {
                    initialPos = "A2";
                }
                if (isStarted())break;

            }
        }



        imu.resetYaw();
        
        
        
        LEDWhite1.setState(true);
        LEDWhite2.setState(true);
        LEDRed1.setState(true);
        LEDRed2.setState(false);
        LEDGreen1.setState(false);
        LEDGreen2.setState(false);
        LEDBlue1.setState(true);
        LEDBlue2.setState(true);
        
        /* INITIALIZE ROBOT - SIGNAL INITIALIZATION COMPLETE */
        // Report initialization complete
        telemetry.addData("Initial Position:",initialPos);
        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Hold for Start");
        telemetry.update();



        // WAIT FOR THE GAME TO START (driver presses PLAY)
        waitForStart();

        autoTimer.reset();  // Reset/restart the autotimer

        // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS

    }








    /* -------------------------------------------------------
     * Method: driveTrainTelemetry
     * Purpose: Report the position and speed of the drive train wheels
     * ------------------------------------------------------- */

    void DriveTrainTelemetry() {

        telemetry.addLine("Top Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.front.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.front.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.front.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.front.motorLeft.targetCount);
        telemetry.addData("  Is Busy", driveTrain.front.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.front.leftSpeed);


        telemetry.addLine("Bottom Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.rear.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.rear.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.rear.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.rear.motorLeft.targetCount);
        telemetry.addData("  Is Busy", driveTrain.rear.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.rear.leftSpeed);

        telemetry.addLine("Right  Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.front.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.front.motorRight.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.front.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.front.motorRight.targetCount);
        telemetry.addData("  Is Busy", driveTrain.front.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.front.rightSpeed);

        telemetry.addLine("Left Motor");
        telemetry.addData("  Position in EngUnits", "%.2f", driveTrain.rear.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits", "%.2f", driveTrain.rear.motorRight.targetPosition);
        telemetry.addData("  Position in Counts", driveTrain.rear.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts", driveTrain.rear.motorRight.targetCount);
        telemetry.addData("  Is Busy", driveTrain.rear.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed", driveTrain.rear.rightSpeed);
    }

    void motorTelemetryDegrees (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in Degrees", "%.2f degrees ", motor.getDegrees());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }

    void motorTelemetry (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in EU", "%.2f EU ", motor.getPosition());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }


    boolean Kill ( double autoTime) {
        boolean eStop;

        if (!opModeIsActive() || autoTimer.seconds() >= autoTime || safeStop) {
            driveTrain.stop();
            //Stops movement of shooter

            eStop = true;
        }
        else
            eStop = false;
        return eStop;
    }

    public void turnToHeading(double heading) {
        int count = 0;
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnTarget = getSteeringCorrection(heading);

            count += 1;
            // Pivot in place by applying the turning correction
            driveTrain.turnCwToTarget (turnTarget, .9);
            while (!driveTrain.isMoveDone(0.25)){
                telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
                telemetry.addData("count",count);
                telemetry.update();
                if(autoTimer.seconds() > 28)
                    break;
            }
            if(autoTimer.seconds() > 28)
                break;
        }

        // Stop all motion;

    }

    public double getSteeringCorrection(double desiredHeading) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Determine the required steering correction
        return(headingError);
    }

    public double getRawHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return -1*orientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public int scanLocation(){
        int location = -1;
        //scann
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if(blocks.length > 0 && (blocks[0].height > 20 && blocks[0].width > 20)){
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                int x = blocks[i].left;
                telemetry.addData("X",x);

                if(initialPos == "A4" || initialPos == "F2"){
                    if(x > 50 && x<180) location = 1;
                    else if (x<50) location = 0;
                    else location = 2;
                }
                //telemetry.addData("Location A4/F2",location);
                
                
                else if(initialPos == "A2" || initialPos == "F4"){
                    if(x > 50 && x<180 ) location = 1;
                    else if (x>180) location = 2;
                    else location = 0;
                    
                }
                //telemetry.addData("Location A2/F4",location);
            }
        }
        else if (location == -1 && (initialPos == "A4" || initialPos == "F2"))location = 2;
        else if (location == -1 && (initialPos == "A2" || initialPos == "F4"))location = 0;


        telemetry.addData("objectLocation",location);
        telemetry.update();


        return location;
    }
    public void funnyArmMove(){
        clawRotate = clawRotateInt;
        pixelClawRotate.goToPositionNow(clawRotate);
        int armPos = 0;
        pixelArm.setTargetPosition((int)armPos);
        sleep(300);
        for(int i=0;i<24;i++){
            armPos += 10;
            pixelArm.setTargetPosition((int)armPos);
            clawRotate -= .05;
            pixelClawRotate.goToPositionNow(clawRotate);

            if(armPos > 240) armPos = 240;
        }
        sleep(500);
        pixelClawRotate.goToPositionNow(1);
        pixelWrist.goToPositionNow(wristPos-.2);
        sleep(500);
        for (int h=0; h<16;h++){
            armPos += 30;
            pixelArm.setTargetPosition((int)armPos);
            sleep(50);
        }
        pixelClawRotate.goToPositionNow(1);

        sleep(500);
    }

    public void reverseFunnyArmMove(){
        pixelClawRotate.goToPositionNow(0);
        int armPos = 720;
        for (int h=0; h<60;h++){
            armPos -= 8;
            pixelArm.setTargetPosition((int)armPos);
            sleep(50);
        }
        pixelWrist.goToPositionNow(wristPos);
        for(int i=0;i<48;i++){
            armPos -= 5;
            pixelArm.setTargetPosition((int)armPos);
            clawRotate += .03;
            pixelClawRotate.goToPositionNow(clawRotate);
            sleep(125);

            if(armPos < 0) armPos = 0;
            if(clawRotate > clawRotateInt)clawRotate = clawRotateInt;
        }


        pixelClawRotate.goToPositionNow(clawRotateInt);
        sleep(500);
    }
    public void placeOnBackdrop(){
        // 2nd arm movment
        // ArmPos = 730
        // WristPos = .39
        // RotatePos = .99
        pixelArm.setTargetPosition(730);
        sleep(250);
        pixelWrist.goToPositionNow(.39);
        sleep(250);
        pixelClawRotate.goToPositionNow(.99);
        sleep(250);

        boxServoRight.goToPositionNow(0.3);
        sleep(400);
    }

}