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


/**
 * Autonomous Opmode G4
 */

@Autonomous(name="Wired_Woodmen_G4", group="Wired")
//@Disabled
public class Wired_Woodmen_G4 extends AutoLinearAbstract {

    // Declare OpMode members specific to this Autonomous Opmode variant.


    @Override
    public void runOpMode() {

        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        super.runOpMode();


        //moves arm down in order to knock off appropriate jewel.
        telemetry.addLine("Wait - Grip Glyph and Lower jewel arm");
        telemetry.update();
        servoGlyphGripper.goToPositionNow(G_GRIPPER_CLOSED);
        servoJewelArm.goToPosition(J_ARM_DOWN_POS, J_ARM_INC_PER_SCAN);


        //evaluates the colors of both jewels
        generalTimer.reset();
        while (generalTimer.seconds() < 2) {
            colorLeftJewel.evaluateColor();
            colorRightJewel.evaluateColor();

            if (colorLeftJewel.colorRed) {
                leftRedJewel = true;
                break;
            }

            if (colorRightJewel.colorRed) {
                rightRedJewel = true;
                break;
            }

            if (colorLeftJewel.colorBlue) {
                leftBlueJewel = true;
                break;
            }

            if (colorRightJewel.colorBlue) {
                rightBlueJewel = true;
                break;
            }

            jewelColorTelemetry();
            telemetry.update();
        }


        //Jewel Arm2 moves to knock of blue jewel
        telemetry.addLine("Wait - Knock off jewel with Jewel Arm 2");
        telemetry.update();
        if (rightBlueJewel || leftRedJewel)
            servoJewelArm2.goToPosition(J_ARM2_LEFT, J_ARM2_INC_PER_SCAN);
        if (leftBlueJewel || rightRedJewel)
            servoJewelArm2.goToPosition(J_ARM2_RIGHT, J_ARM2_INC_PER_SCAN);


        // Initiate glyph arm move to a safe height for robot travel
        telemetry.addLine("Wait - Raise glyph arm for safe travel");
        jewelColorTelemetry();
        telemetry.update();
        glyphArm.goToAbsoluteDegrees(GLYPH_ARM_TRAVEL_ANGLE, GLYPH_ARM_DEFAULT_SPEED);


        //This raises the robots jewel arm to an intermediate position to allow the JewelArm2 to return to a safe position.
        telemetry.addLine("Wait - Raise jewel arm to intermediate position");
        jewelColorTelemetry();
        telemetry.update();
        servoJewelArm.goToPosition(J_ARM_INTERMEDIATE_POS, J_ARM_INC_PER_SCAN_FAST);


        //Jewel Arm2 Returns to start pos
        telemetry.addLine("Wait - Return Jewel Arm 2 and 1");
        telemetry.update();
        servoJewelArm2.goToPositionNow(J_ARM2_RETURN);
        servoJewelArm.goToPosition(J_ARM_UP_POS, J_ARM_INC_PER_SCAN);


        // Go straight to exit balancing stone
        driveTrain.goStraightToTarget(24,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight to exit balancing stone");
            driveTrainTelemetry();
            jewelColorTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        // rotate to face the crypto box
        driveTrain.turnCcwToTarget(17,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive rotate to face cryptobox");
            driveTrainTelemetry();
            jewelColorTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        //go strait to cryptobox
        driveTrain.goStraightToTarget(39,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move #1 to crypto box");
            driveTrainTelemetry();
            jewelColorTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 26) {
                driveTrain.stop();
                break;
            }
        }


        //this lowers the glyph arm to zero degrees
        glyphArm.goToAbsoluteDegrees(0, GLYPH_ARM_DEFAULT_SPEED);
        while (!glyphArm.isMoveDone(MAX_GLYPH_ARM_POSITION_ERROR_DEGREES)) {
            telemetry.addLine("Wait - Lower glyph arm to home position");
            jewelColorTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                glyphArm.stop();
                break;
            }
        }


        // Go straight to deliver glyph
        driveTrain.goStraightToTarget(3,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Wait - Drive train move #2 to crypto box");
            driveTrainTelemetry();
            jewelColorTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


        //this will release the glyph given to us in pre-match
        telemetry.addLine("Wait - Open glyph grippers");
        driveTrainTelemetry();
        jewelColorTelemetry();
        telemetry.update();
        servoGlyphGripper.goToPosition(G_GRIPPER_OPEN,G_GRIPPER_INC_PER_SCAN);


        // Reverse from glyph
        driveTrain.goStraightToTarget(-1,DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move #3 'to crypto box");
            driveTrainTelemetry();
            jewelColorTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


        telemetry.addLine("Program Complete");
        telemetry.update();
    }
}
