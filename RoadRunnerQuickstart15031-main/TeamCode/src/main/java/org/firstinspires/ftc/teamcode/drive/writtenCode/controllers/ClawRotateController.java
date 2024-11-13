package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ClawRotateController {
    public enum ClawRotateStatus{
        INIT,
        RUNTO;
    }
    public ClawRotateStatus currentStatus = ClawRotateStatus.INIT;
    public ClawRotateStatus previousStatus=null;
    public static double init_position=0;
    public Servo clawRotate = null;
    public ClawRotateController(RobotMap robot) {
        this.clawRotate=robot.clawRotate;
    }
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus == ClawRotateStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.clawRotate.setPosition(init_position);
                    break;
                }
            }
        }
    }
}
