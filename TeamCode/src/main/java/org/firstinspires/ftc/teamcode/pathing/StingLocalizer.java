package org.firstinspires.ftc.teamcode.pathing;


import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//an interface so that we can switch localizers out in our code as much as we want, to test them against each other
public interface StingLocalizer {
    double[] getPoseDouble();
    double[] getVelocity();
    void setPose(Pose2D pose);
    Pose2D getPose();
    void update();
}
