// Import FTC package
package org.firstinspires.ftc.teamcode;

// Import FTC classes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// Import custom-made classes/methods
import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;

public class DriveOpMode extends OpMode {
    // Create variables
    DcMotor frontLeft,frontRight,backLeft,backRight;
    double[] motorPowers;
    DcMotor[] driveMotors = {frontLeft, frontRight, backLeft, backRight};

    @Override
    // Set starting values for variables
    public void init() {
        // Map variables to motors
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set universal wheel behaviors
        for (DcMotor i : driveMotors) i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Gets power levels for each motor, using gamepad inputs as directions
        // The third item in the array dictates which trigger is being pressed (=1 if left, =-1 if right, =0 if none or both).
        motorPowers = MecanumKinematics.getPowerFromDirection(new double[] {
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                toInt(gamepad1.left_bumper) - toInt(gamepad1.right_bumper)
        });

        // Sets power levels
        // Works because each index corresponds with the same wheel in both arrays
        for (int i=0; i < motorPowers.length; i++) {
            driveMotors[i].setPower(motorPowers[i]);
        };
    }

}
