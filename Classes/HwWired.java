package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is a hardware class for the Wired Woodman
 *
 * Revised 13Feb2018 - Modified Glyph Arm speed and limits; set glyph arm zero power behavior to BRAKE
 * Revised 15Feb2018 - Removed glyph arm ZeroPowerBehavior
 */

public class HwWired
{
    /* Public OpMode members. */
    DcMotor
            leftMotor = null,
            rightMotor = null,
            glyphArm = null,
            relicRaiser = null,
            relicExtender = null;

    Servo
            relicGrabber = null,
            jewelArm = null,
            jewelArm2 = null,
            glyphGripper = null,
            relicLinear = null;

    final double
            J_ARM_UP_POS =  0.6,
            J_ARM_STRAIGHT_POS =  0.1,
            J_ARM2_INIT_POS = 0.5,
            G_GRIPPER_OPEN = 0.6,
            G_GRIPPER_CLOSED = 1.0,
            R_GRABBER_CLOSED_POS = 0.85,
            R_GRABBER_OPEN_POS = 0.3,
            R_LINEAR_OUT_POS = 0.9,
            R_LINEAR_IN_POS = 0.1,

            G_ARM_SPEED_FACTOR = 0.2,
            R_ARM_SPEED_FACTOR = 0.65,
            R_EXTENDER_SPEED_FACTOR = 0.55,
            R_EXTENDER_RETRACT_SPEED_FACTOR = 4,
            R_LINEAR_INC_PER_SCAN = 0.001;

    final int
            G_ARM_UP_POS =  1250,
            G_ARM_SAFE_POS = 725,
            G_ARM_DOWN_POS =  0,
            R_ARM_UP_POS =  17000,
            R_ARM_SAFE_POS = 2700,
            R_ARM_DOWN_POS = 0,
            R_ARM_OUT_POS = 8550,
            R_ARM_IN_POS = 0;

    /* local OpMode members. */
    HardwareMap hwMap =  null;


    /* Constructor */
    public HwWired(){
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;


        // Identify and link connected hardware
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        glyphArm    = hwMap.dcMotor.get("glyph_arm");
        relicRaiser = hwMap.dcMotor.get ("relic_raiser");
        relicExtender=hwMap.dcMotor.get ("relic_extender");

        jewelArm = hwMap.servo.get("jewel_arm");
        jewelArm2 = hwMap.servo.get("jewelarm2");
        glyphGripper =  hwMap.servo.get("glyph_gitter");
        relicGrabber =  hwMap.servo.get("relic_grabber");
        relicLinear =  hwMap.servo.get("relic_linear");


        // Configure and Initialize Motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        glyphArm.setDirection(DcMotor.Direction.REVERSE);
        glyphArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        relicRaiser.setDirection(DcMotor.Direction.FORWARD);
        relicRaiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicRaiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        relicExtender.setDirection(DcMotor.Direction.FORWARD);
        relicExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        glyphArm.setPower(0);
        relicRaiser.setPower(0);
        relicExtender.setPower(0);


        // Initialize servo positions.
        jewelArm.setPosition(J_ARM_UP_POS);
        jewelArm2.setPosition(J_ARM2_INIT_POS);

        glyphGripper.setPosition(G_GRIPPER_OPEN);

        relicGrabber.setPosition(R_GRABBER_OPEN_POS);
        relicLinear.setPosition(R_LINEAR_IN_POS);
    }

}

