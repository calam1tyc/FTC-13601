package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumRobot extends Robot {
    private final DcMotor leftFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor rightBackMotor;
    private final DcMotor[] motors;

    public MecanumRobot(HardwareMap hardwareMap) {
        super(4, hardwareMap);
        leftFrontMotor = lDriveMotors[0];
        leftBackMotor = lDriveMotors[1];
        rightFrontMotor = rDriveMotors[0];
        rightBackMotor = rDriveMotors[1];
        motors = new DcMotor[]{leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor};
    }

    public void diagonalMoveInches(float xInches, float yInches, float speed) {
        float diagonal = (float)Math.sqrt(xInches * xInches + yInches * yInches);
        float leftFrontPower = yInches + xInches;
        float leftBackPower = yInches - xInches;
        float rightFrontPower = yInches - xInches;
        float rightBackPower = yInches + xInches;
        float[] powers = new float[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        max /= speed;

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition
                    (motors[i].getCurrentPosition() + (int)(diagonal * COUNTS_PER_INCH) * (int)(powers[i]/Math.abs(powers[i])));
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(powers[i]);
        }

        while (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && rightBackMotor.isBusy()) {
            telemetry.addData("Moving", true);
            telemetry.update();
        }

        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(0);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turnDeg(float deg, float speed) {
        float inches = DIAMETER_INCHES * (float)Math.PI / (float)(WHEEL_DIAMETER_INCHES * Math.PI) * deg/360f;

        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH));
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH));

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(speed));
        }

        while (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && rightBackMotor.isBusy()) {
            telemetry.addData("Turning", true);
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turnToMoveInches(float xInches, float yInches, float turnSpeed, float speed) {
        float deg = (float)(Math.atan(xInches/yInches) * 180/Math.PI);
        turnDeg(deg, turnSpeed);
        float diagonal = (float)(Math.sqrt(xInches * xInches + yInches * yInches));
        for (DcMotor motor:motors) {
            motor.setTargetPosition(motor.getCurrentPosition() + (int)(diagonal * COUNTS_PER_INCH));
            motor.setPower(speed);
        }
        while (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && rightBackMotor.isBusy()) {
            telemetry.addData("Moving", true);
            telemetry.update();
        }
        for (DcMotor motor:motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        turnDeg(-deg, turnSpeed);
    }
}
