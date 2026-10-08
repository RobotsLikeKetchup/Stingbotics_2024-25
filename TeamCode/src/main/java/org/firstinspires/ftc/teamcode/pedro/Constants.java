package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
    public static MecanumConfig driveConfig = new MecanumConfig(
            config ->{
                config.frontLeftName.set("motor_fl");
                config.backLeftName.set("motor_bl");
                config.frontRightName.set("motor_fr");
                config.backRightName.set("motor_br");
                config.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                config.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                config.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                config.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );
}