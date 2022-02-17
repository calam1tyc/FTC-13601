package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    public DcMotor[] lDriveMotors;
    public DcMotor[] rDriveMotors;
    private ElapsedTime movementTime;
    private HardwareMap hwMap;

    static final float DIAMETER_INCHES = 18;
    static final int COUNTS_PER_MOTOR_REV = 1440; //?
    static final float DRIVE_GEAR_REDUCTION = 2.0f; //?
    static final float WHEEL_DIAMETER_INCHES = 6.0f; //?
    static final float COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415f);
    static final float DRIVE_SPEED = 0.6f;
    static final float TURN_SPEED = 0.5f;

    //lDriveMotors[0] is front, lDriveMotors[n] is back
    public Robot(int motors, HardwareMap hardwareMap) {
        hwMap = hardwareMap;
        movementTime = new ElapsedTime();
        lDriveMotors = new DcMotor[motors/2];
        rDriveMotors = new DcMotor[motors/2];

        for (int i = 0; i < motors; i++) {
            lDriveMotors[i] = hwMap.get(DcMotor.class, "leftDrive" + (i + 1));
            rDriveMotors[i] = hwMap.get(DcMotor.class, "rightDrive" + (i + 1));

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
    public void basicMove(float speed, float inches) {
        int leftTarget = lDriveMotors[0].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        int rightTarget = rDriveMotors[0].getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        lDriveMotors[0].setTargetPosition(leftTarget);
        rDriveMotors[0].setTargetPosition(rightTarget);

        lDriveMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDriveMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < lDriveMotors.length; i++) {
            lDriveMotors[i].setPower(Math.abs(speed));
            rDriveMotors[i].setPower(Math.abs(speed));
        }

        while (lDriveMotors[0].isBusy() && rDriveMotors[0].isBusy()) {
            telemetry.addData("Path1",  "Running to %7d :%7d", leftTarget,  rightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    lDriveMotors[0].getCurrentPosition(),
                    rDriveMotors[0].getCurrentPosition());
            telemetry.update();
        }

        for (int i = 0; i < lDriveMotors.length; i++) {
            lDriveMotors[i].setPower(0);
            rDriveMotors[i].setPower(0);
            lDriveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rDriveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turn(float speed, float deg) {
        float inches = DIAMETER_INCHES * (float)Math.PI / (WHEEL_DIAMETER_INCHES * (float)Math.PI) * deg/360f;
        int leftTarget = lDriveMotors[0].getCurrentPosition() + (int)(inches * deg/360f);
        int rightTarget = rDriveMotors[0].getCurrentPosition() - (int)(inches * deg/360f);

        lDriveMotors[0].setTargetPosition(leftTarget);
        rDriveMotors[0].setTargetPosition(rightTarget);

        lDriveMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDriveMotors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < lDriveMotors.length; i++) {
            lDriveMotors[i].setPower(Math.abs(speed));
            rDriveMotors[i].setPower(Math.abs(speed));
        }

        while (lDriveMotors[0].isBusy() && rDriveMotors[0].isBusy()) {
            telemetry.addData("Path1",  "Running to %7d :%7d", leftTarget,  rightTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    lDriveMotors[0].getCurrentPosition(),
                    rDriveMotors[0].getCurrentPosition());
            telemetry.update();
        }

        for (int i = 0; i < lDriveMotors.length; i++) {
            lDriveMotors[i].setPower(0);
            rDriveMotors[i].setPower(0);
            lDriveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rDriveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
