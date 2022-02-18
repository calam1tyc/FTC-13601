package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public DcMotor[] lDriveMotors;
    public DcMotor[] rDriveMotors;

    static final float DIAMETER_INCHES = 18;
    static final float DRIVE_SPEED = 0.6f;
    static final float TURN_SPEED = 0.5f;

    //lDriveMotors[0] is front, lDriveMotors[n] is back
    public Robot(int motors, HardwareMap hardwareMap) {
        lDriveMotors = new DcMotor[motors/2];
        rDriveMotors = new DcMotor[motors/2];

        for (int i = 0; i < motors; i++) {
            lDriveMotors[i] = hardwareMap.get(DcMotor.class, "leftDrive" + (i + 1));
            rDriveMotors[i] = hardwareMap.get(DcMotor.class, "rightDrive" + (i + 1));

            lDriveMotors[i].setDirection(DcMotor.Direction.FORWARD);
            rDriveMotors[i].setDirection(DcMotor.Direction.REVERSE);

            lDriveMotors[i].setPower(0);
            rDriveMotors[i].setPower(0);

            lDriveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rDriveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetEncoders() {
        for (int i = 0; i < lDriveMotors.length; i++) {
            lDriveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rDriveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lDriveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rDriveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

