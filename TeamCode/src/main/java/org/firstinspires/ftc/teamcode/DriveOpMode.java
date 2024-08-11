// Import FTC package
package org.firstinspires.ftc.teamcode;

// Import FTC classes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Import custom-made classes/methods
import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;
import org.firstinspires.ftc.teamcode.hardware.Robot;

//This opMode uses the standardconfig configuration file.
@TeleOp

public class DriveOpMode extends OpMode {
    // Create variables
    DcMotor frontLeft,frontRight,backLeft,backRight;
    double[] motorPowers;
    DcMotor[] driveMotors;

    Robot robot = new Robot();

    @Override
    // Set starting values for variables
    public void init() {
        robot.init(hardwareMap);

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Gets power levels for each motor, using gamepad inputs as directions
        // The third item in the array dictates which trigger is being pressed (=1 if left, =-1 if right, =0 if none or both).
        motorPowers = MecanumKinematics.getPowerFromDirection(new double[] {
                -1 * gamepad1.left_stick_x, //multiply by negative 1 to reverse
                gamepad1.left_stick_y,
                toInt(gamepad1.right_bumper) - toInt(gamepad1.left_bumper)
        });

        // Sets power levels
        // Works because each index corresponds with the same wheel in both arrays
        for (int i=0; i < motorPowers.length; i++) {
            driveMotors[i].setPower(motorPowers[i]);
        };
    }

}
