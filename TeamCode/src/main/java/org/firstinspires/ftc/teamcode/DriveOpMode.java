// Import FTC package
package org.firstinspires.ftc.teamcode;

// Import FTC classes
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Import custom-made classes/methods
import static org.firstinspires.ftc.teamcode.utilities.MathFunctions.toInt;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pathing.EzraLocalizer;

//This opMode uses the standardconfig configuration file.
@TeleOp

public class DriveOpMode extends OpMode {
    // Create variables
    double[] motorPowers;

    Robot robot = new Robot();

    ElapsedTime timer= new ElapsedTime();

    EzraLocalizer localizer = new EzraLocalizer(robot,new double[] {0,0,0},timer);

    @Override
    // Set starting values for variables
    public void init() {
        robot.init(hardwareMap);
        localizer.setWheelDistances(16.5,8);
        localizer.init();

        // Update telemetry (for feedback)
        telemetry.addLine("Initialized!");
        telemetry.addData("X pos",localizer.getPose()[0]);
        telemetry.addData("y pos",localizer.getPose()[1]);
        telemetry.addData("Angle pos",localizer.getPose()[2]);
        telemetry.update();
    }

    @Override
    public void loop() {
        localizer.calcPose();
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
            robot.driveMotors[i].setPower(motorPowers[i]);
        };
        telemetry.addData("X pos",localizer.getPose()[0]);
        telemetry.addData("y pos",localizer.getPose()[1]);
        telemetry.addData("Angle pos",localizer.getPose()[2]);
        telemetry.addData("Encoder 1", robot.getDeadwheel("parR").getTicks());
        telemetry.addData("Encoder 1", robot.getDeadwheel("parL").getTicks());
        telemetry.addData("Encoder 1", robot.getDeadwheel("per").getTicks());
        telemetry.update();
    }

}
