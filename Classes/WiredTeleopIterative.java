/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Primary Wired Woodman Teleop Program
 *
 * Revised 13Feb2018 - Added GlyphArm/RelicArm protection; cleaned up code organization
 * Revised 15Feb2018 - Removed glyph arm ZeroPowerBehavior and coded a new brake solution
 */

@TeleOp(name="Wired Woodman", group="Wired")
//@Disabled
public class WiredTeleopIterative extends OpMode{

    /* Declare OpMode members. */
    HwWired robot = new HwWired(); // New Robot object created from the HwWired class

    private double
            left = 0,
            right = 0,
            glyphArmSpeed = 0,
            relicRaiserSpeed = 0,
            glyphGripperPosition = robot.G_GRIPPER_OPEN,
            relicLinearPosition = robot.R_LINEAR_OUT_POS;

    private boolean
            brakeGlyphArm = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(hardwareMap); // Initialize the robot object via it's init method
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.glyphArm.setPower(0);
        robot.relicRaiser.setPower(0);
        robot.relicExtender.setPower(0);

        telemetry.addLine("Say Hello Driver");  // Send message to signify robot waiting
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);


        // Raise/lower jewel arm with dpad up/down
        if (gamepad1.dpad_up)
            robot.jewelArm.setPosition(robot.J_ARM_UP_POS);

        if (gamepad1.dpad_down)
            robot.jewelArm.setPosition(robot.J_ARM_STRAIGHT_POS);


        // Raise/lower glyph arm with left/right triggers
        if (gamepad1.left_trigger >= 0.10) {
            if (robot.relicRaiser.getCurrentPosition() > robot.R_ARM_SAFE_POS) // Limit glyph arm motion if relic are not in safe position
                robot.glyphArm.setTargetPosition(robot.G_ARM_SAFE_POS);
            else
                robot.glyphArm.setTargetPosition(robot.G_ARM_UP_POS);
            glyphArmSpeed = gamepad1.left_trigger * robot.G_ARM_SPEED_FACTOR;
            robot.glyphArm.setPower(glyphArmSpeed);
        }

        if (gamepad1.right_trigger >= 0.10) {
            robot.glyphArm.setTargetPosition(robot.G_ARM_DOWN_POS);
            glyphArmSpeed = gamepad1.right_trigger * robot.G_ARM_SPEED_FACTOR;
            robot.glyphArm.setPower(glyphArmSpeed);
        }

        if (gamepad1.left_trigger < 0.10 && gamepad1.right_trigger < 0.10) {
            if (!brakeGlyphArm) {
                brakeGlyphArm = true;
                robot.glyphArm.setTargetPosition(robot.glyphArm.getCurrentPosition());
                robot.glyphArm.setPower(robot.G_ARM_SPEED_FACTOR);
            }
        }
        else
            brakeGlyphArm = false;


        // Open/Close glyph gripper with left/right bumpers
        if (gamepad1.left_bumper) {
            glyphGripperPosition = glyphGripperPosition - 0.05;
            if (glyphGripperPosition < robot.G_GRIPPER_OPEN)
                glyphGripperPosition = robot.G_GRIPPER_OPEN;
        }

        if (gamepad1.right_bumper) {
            glyphGripperPosition = glyphGripperPosition + 0.05;
            if (glyphGripperPosition > robot.G_GRIPPER_CLOSED)
                glyphGripperPosition = robot.G_GRIPPER_CLOSED;
        }

        robot.glyphGripper.setPosition(glyphGripperPosition);


        // Raise/lower relic arm with left/right triggers
        if (gamepad2.left_trigger >= 0.10 && robot.glyphArm.getCurrentPosition()< robot.G_ARM_SAFE_POS) {
            // motion is prevented if glyph arm is too high (i.e., not safe)
            robot.relicRaiser.setTargetPosition(robot.R_ARM_UP_POS);
            relicRaiserSpeed = gamepad2.left_trigger * robot.R_ARM_SPEED_FACTOR;
            robot.relicRaiser.setPower(relicRaiserSpeed);
        }

        if (gamepad2.right_trigger >= 0.10) {
            robot.relicRaiser.setTargetPosition(robot.R_ARM_DOWN_POS);
            relicRaiserSpeed = gamepad2.right_trigger * robot.R_ARM_SPEED_FACTOR;
            robot.relicRaiser.setPower(relicRaiserSpeed);
        }

        if (gamepad2.left_trigger < 0.10 && gamepad2.right_trigger < 0.10)
            robot.relicRaiser.setPower(0);


        // Extend/retract relic arm with left/right bumpers
        if (gamepad2.left_bumper) {
            robot.relicExtender.setTargetPosition(robot.R_ARM_OUT_POS);
            robot.relicExtender.setPower(robot.R_EXTENDER_SPEED_FACTOR);
        }

        if (gamepad2.right_bumper) {
            robot.relicExtender.setTargetPosition(robot.R_ARM_IN_POS);
            robot.relicExtender.setPower(robot.R_EXTENDER_RETRACT_SPEED_FACTOR);
        }

        if (!gamepad2.left_bumper && !gamepad2.right_bumper)
            robot.relicExtender.setPower(0);


        // Extend/retract secondary relic arm with A/Y buttons
        if (gamepad2.a) {
            relicLinearPosition = relicLinearPosition - robot.R_LINEAR_INC_PER_SCAN;
            if (relicLinearPosition < robot.R_LINEAR_IN_POS)
                relicLinearPosition = robot.R_LINEAR_IN_POS;
        }

        if (gamepad2.y) {
            relicLinearPosition = relicLinearPosition + robot.R_LINEAR_INC_PER_SCAN;
            if (relicLinearPosition > robot.R_LINEAR_OUT_POS)
                relicLinearPosition = robot.R_LINEAR_OUT_POS;
        }

        robot.relicLinear.setPosition(relicLinearPosition);


        // Open/close relic gripper with dpad up/down
        if (gamepad2.dpad_up) {
            robot.relicGrabber.setPosition(robot.R_GRABBER_OPEN_POS);
        }

        if (gamepad2.dpad_down) {
            robot.relicGrabber.setPosition(robot.R_GRABBER_CLOSED_POS);
        }


        // Provide critical data to the Driver Station phone
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("G Gripper", glyphGripperPosition);
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);
        telemetry.addData("G Arm Position", robot.glyphArm.getCurrentPosition());
        telemetry.addData("R Raiser Position", robot.relicRaiser.getCurrentPosition());
        telemetry.addData("R Extender Position", robot.relicExtender.getCurrentPosition());
        telemetry.addData("R Linear Setpoint", "%.2f",relicLinearPosition);
        telemetry.addData("gamepad2",gamepad2.right_trigger);
        telemetry.addData("R Raise Speed",relicRaiserSpeed);
        telemetry.addData("R Linear Position",relicLinearPosition);
        telemetry.addData("brake Glyph", brakeGlyphArm);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Set all motors to zero power
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        robot.glyphArm.setPower(0);
        robot.relicRaiser.setPower(0);
        robot.relicExtender.setPower(0);
    }

}
