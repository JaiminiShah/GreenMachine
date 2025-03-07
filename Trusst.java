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

import android.animation.IntArrayEvaluator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Trusst", group="Autonomous")
public class Trusst extends AutoLinearAbstract2023 {

    // Declare OpMode members specific to this Autonomous Opmode variant.

    @Override
    public void runOpMode() {

        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        RunAutoInput = true;
        super.runOpMode();
        pixelArm.setTargetPosition(90);
        sleep(300);


        if(initialPos == "A4"){
            driveTrain.StrafeLeftToTarget(10, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }

            driveTrain.goStraightToTarget(16, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
            intake.setPower(-0.7);
            sleep(1000);
            intake.setPower(0);
            pixelArm.setTargetPosition(90);

            driveTrain.goStraightToTarget(-16, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
            driveTrain.StrafeLeftToTarget(48, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }


        }
        else if(initialPos == "F4"){
            driveTrain.StrafeRightToTarget(10, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }

            driveTrain.goStraightToTarget(16, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
            intake.setPower(-0.7);
            sleep(1000);
            intake.setPower(0);
            pixelArm.setTargetPosition(90);

            driveTrain.goStraightToTarget(-16, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
            driveTrain.StrafeRightToTarget(48, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }

        }
        else if(initialPos == "A2"){

            driveTrain.StrafeRightToTarget(10, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }

            driveTrain.goStraightToTarget(16, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
            intake.setPower(-0.7);
            sleep(1000);
            intake.setPower(0);
            pixelArm.setTargetPosition(90);

            driveTrain.goStraightToTarget(-13, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
            driveTrain.StrafeLeftToTarget(96, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }

        }
        else if(initialPos == "F2") {
            driveTrain.StrafeLeftToTarget(10, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }

            driveTrain.goStraightToTarget(16, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
            intake.setPower(-0.7);
            sleep(1000);
            intake.setPower(0);
            pixelArm.setTargetPosition(90);

            driveTrain.goStraightToTarget(-13, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
            driveTrain.StrafeRightToTarget(96, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }
        }
        else{
            driveTrain.goStraightToTarget(36, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Going to place pixel");
                if (Kill(28)) {
                    break;
                }
            }

            intake.setPower(-0.7);
            sleep(1000);
            intake.setPower(0);
            pixelArm.setTargetPosition(90);
        }




        telemetry.addLine("Autonomous Done");
        telemetry.update();
    }
}




