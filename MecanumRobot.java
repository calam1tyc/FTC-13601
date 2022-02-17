package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumRobot extends Robot {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor[] motors;

    public MecanumRobot(HardwareMap hardwareMap) {
        super(4, hardwareMap);
        leftFrontMotor = lDriveMotors[0];
        leftBackMotor = lDriveMotors[1];
        rightFrontMotor = rDriveMotors[0];
        rightBackMotor = rDriveMotors[1];
        motors = new DcMotor[]{leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor};
    }

    public void moveInches(float xInches, float yInches) {
        float diagonal = (float)Math.sqrt(xInches * xInches + yInches * yInches);
        float leftFrontPower = yInches + xInches;
        float leftBackPower = yInches - xInches;
        float rightFrontPower = yInches - xInches;
        float rightBackPower = yInches + xInches;
        float[] powers = new float[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        max /= Robot.DRIVE_SPEED;

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
            telemetry.update();
        }

        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(0);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
