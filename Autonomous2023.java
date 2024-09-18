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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Objects;

@Autonomous(name="Autonomous2023", group="Autonomous")
public class Autonomous2023 extends AutoLinearAbstract2023 {

    // Declare OpMode members specific to this Autonomous Opmode variant.

    @Override
    public void runOpMode() {

        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        RunAutoInput = true;
        super.runOpMode();
        //int ARM_UP = 180;

        pixelArm.setTargetPosition(10);
        objectLocation = scanLocation();
        //Now to take a nice cozy sleep zzzzz
        sleep(600);
        telemetry.addData("objectLocation",objectLocation);
        telemetry.update();


        //auto begin
        if(Objects.equals(initialPos, "F4")){
            turnToHeading(0);

            if (objectLocation == 0){
                driveTrain.goStraightToTarget(24.5, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
                driveTrain.goStraightToTarget(6, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
            
                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);
                
                driveTrain.goStraightToTarget(-10, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }

               //funnyArmMove();

                turnToHeading(-90);

                driveTrain.goStraightToTarget(-30, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                driveTrain.StrafeRightToTarget(7, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);
                
                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                
                
                driveTrain.StrafeRightToTarget(19, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }

                //reverseFunnyArmMove();
                driveTrain.goStraightToTarget(-14, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                    
                
            }
// end of pos 0
//start pos 1
            else if (objectLocation == 1){
                driveTrain.goStraightToTarget(28, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(0);

                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);

                driveTrain.goStraightToTarget(-4, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                turnToHeading(-90);
               //funnyArmMove();
                driveTrain.goStraightToTarget(-36, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                driveTrain.StrafeRightToTarget(7, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeRightToTarget(28, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }

            else if (objectLocation == 2) {
                driveTrain.goStraightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(0);
                driveTrain.StrafeRightToTarget(10, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }

                
                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);

                driveTrain.goStraightToTarget(-4, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }

                turnToHeading(-90);
               //funnyArmMove();
                driveTrain.goStraightToTarget(-34, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                driveTrain.StrafeLeftToTarget(4, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeRightToTarget(34, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }

        }



        if(Objects.equals(initialPos, "A4")){
            turnToHeading(0);

            if (objectLocation == 0){
                driveTrain.StrafeLeftToTarget(24.5, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(0);
                driveTrain.goStraightToTarget(25, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                
                
                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);

                driveTrain.goStraightToTarget(-4, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                

                turnToHeading(90);
               //funnyArmMove();
                
                driveTrain.goStraightToTarget(-20, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                driveTrain.StrafeRightToTarget(9, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeLeftToTarget(36, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }

            else if (objectLocation == 1){
                driveTrain.goStraightToTarget(28, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(0);

                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);

                driveTrain.goStraightToTarget(-4, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }

                turnToHeading(90);

               //funnyArmMove();

                turnToHeading(90);
                
                driveTrain.goStraightToTarget(-36, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                driveTrain.StrafeLeftToTarget(6, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeLeftToTarget(28, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }

            else if (objectLocation == 2) {
                driveTrain.goStraightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                
                driveTrain.goStraightToTarget(5, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);

                driveTrain.goStraightToTarget(-4, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }

                turnToHeading(90);
               //funnyArmMove();
                

                turnToHeading(90);
                
                driveTrain.goStraightToTarget(-30, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                driveTrain.StrafeRightToTarget(3, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(-2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }


                driveTrain.StrafeLeftToTarget(35, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }

        }




        else if(Objects.equals(initialPos, "A2")){
            if (objectLocation == 0){
                driveTrain.goStraightToTarget(28, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
                driveTrain.goStraightToTarget(5, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);

                driveTrain.goStraightToTarget(-5, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                turnToHeading(-90);
                driveTrain.StrafeRightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                driveTrain.goStraightToTarget(45, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
               //funnyArmMove();
                driveTrain.goStraightToTarget(-45, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeRightToTarget(33, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }
            else if (objectLocation == 1){
                driveTrain.goStraightToTarget(42, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                
                turnToHeading(180);



                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);
                
                driveTrain.goStraightToTarget(-4, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                

                turnToHeading(-90);

                driveTrain.goStraightToTarget(45, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
               //funnyArmMove();
                driveTrain.goStraightToTarget(-39, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                
                driveTrain.StrafeLeftToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeRightToTarget(32, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }
            else if (objectLocation == 2) {
                driveTrain.goStraightToTarget(28, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                driveTrain.goStraightToTarget(3, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);

                driveTrain.goStraightToTarget(-3, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                driveTrain.StrafeLeftToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                driveTrain.goStraightToTarget(45, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
               //funnyArmMove();
                driveTrain.goStraightToTarget(-39, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);


                driveTrain.StrafeRightToTarget(23, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeLeftToTarget(26, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }
        }

        else if(Objects.equals(initialPos, "F2")){
            if (objectLocation == 0){
                driveTrain.goStraightToTarget(28, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                turnToHeading(-90);
                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);


                turnToHeading(-90);
                driveTrain.StrafeRightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                driveTrain.goStraightToTarget(45, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
               //funnyArmMove();
                driveTrain.goStraightToTarget(-39, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                driveTrain.StrafeLeftToTarget(17, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }


                driveTrain.StrafeRightToTarget(33, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }
            else if (objectLocation == 1){
                driveTrain.goStraightToTarget(42, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                
                turnToHeading(180);

                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);
                
                driveTrain.goStraightToTarget(-4, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                

                turnToHeading(90);

                driveTrain.goStraightToTarget(45, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
               //funnyArmMove();
                driveTrain.goStraightToTarget(-39, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                driveTrain.StrafeLeftToTarget(26, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }

                turnToHeading(90);
                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeRightToTarget(32, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
            }
            else if (objectLocation == 2){
                driveTrain.goStraightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);
                
                driveTrain.goStraightToTarget(3, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                sleep(200);
                intake.setPower(-1);
                sleep(500);
                intake.setPower(0);

                driveTrain.goStraightToTarget(-3, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                driveTrain.StrafeLeftToTarget(26, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(90);

                driveTrain.goStraightToTarget(45, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
               //funnyArmMove();
                driveTrain.goStraightToTarget(-39, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on backdrop");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                driveTrain.StrafeRightToTarget(23, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);

                turnToHeading(-90);
                boxServoLeft.goToPositionNow(boxServoLeftOpen);
                boxServoRight.goToPositionNow(boxServoRightOpen);
                sleep(500);

                driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to place pixel on spike");
                    if (Kill(28)) {
                        break;
                    }
                }

                driveTrain.StrafeRightToTarget(26, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }
                turnToHeading(-90);
                //reverseFunnyArmMove();

                driveTrain.goStraightToTarget(-12, DRIVE_TRAIN_DEFAULT_SPEED);
                while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                    telemetry.addLine("Going to park");
                    if (Kill(28)) {
                        break;
                    }
                }

            }
        }




        telemetry.addLine("Autonomous Done");
        telemetry.update();
    }
}




