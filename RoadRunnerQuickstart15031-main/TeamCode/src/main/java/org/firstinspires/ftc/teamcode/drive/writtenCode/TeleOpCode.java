package org.firstinspires.ftc.teamcode.drive.writtenCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ArmController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawPositionController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawRotateController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CollectScoreController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ColorController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ExtenderController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.FourbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeController;

@TeleOp(name="TeleOpCode", group="Linear OpMode")
public class TeleOpCode extends LinearOpMode {
    public void setMotorRunningMode(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                    DcMotor rightBack, DcMotor.RunMode runningMode) {
        leftFront.setMode(runningMode);
        rightFront.setMode(runningMode);
        leftBack.setMode(runningMode);
        rightBack.setMode(runningMode);
    }

    public void setMotorZeroPowerBehaviour(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                           DcMotor rightBack, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack,
                                  DcMotor rightFront, DcMotor rightBack,
                                  double leftTrigger, double rightTrigger) {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = (-gamepad1.left_trigger + gamepad1.right_trigger) * 1.05; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;


        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    ElapsedTime GlobalTimer = new ElapsedTime();
    ElapsedTime ArmToExtender = new ElapsedTime();
    double time_between_arm_extender = 1;
    int extenderTargetPosition = 0;
    int armTargetPosition = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotMap robot= new RobotMap(hardwareMap);

        ClawPositionController clawPositionController = new ClawPositionController(robot);
        ClawRotateController clawRotateController = new ClawRotateController(robot);
        ClawController clawController = new ClawController(robot);
        FourbarController fourbarController = new FourbarController(robot);
        IntakeController intakeController = new IntakeController(robot);
        ExtenderController extenderController = new ExtenderController(robot);
        ArmController armController = new ArmController(robot);
        CollectScoreController collectScoreController = new CollectScoreController(armController,extenderController,fourbarController,clawPositionController,robot);
        ColorController colorController = new ColorController(robot);

        clawPositionController.update();
        clawRotateController.update();
        clawController.update();
        extenderController.update(extenderTargetPosition);
        fourbarController.update();
        armController.update(armTargetPosition);
        intakeController.update();
        colorController.update();
        collectScoreController.update();

        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        MotorConfigurationType mct1, mct2, mct3, mct4;
        mct1 = rightBack.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        rightBack.setMotorType(mct1);

        mct2 = rightFront.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        rightFront.setMotorType(mct2);

        mct3 = leftFront.getMotorType().clone();
        mct3.setAchieveableMaxRPMFraction(1.0);
        leftFront.setMotorType(mct3);

        mct4 = leftBack.getMotorType().clone();
        mct4.setAchieveableMaxRPMFraction(1.0);
        leftBack.setMotorType(mct4);
        setMotorRunningMode(leftFront,leftBack,rightFront,rightBack,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        setMotorZeroPowerBehaviour(leftFront,leftBack,rightFront,rightBack,
                DcMotor.ZeroPowerBehavior.BRAKE);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();
        GlobalTimer.reset();
        while(opModeIsActive())
        {
            if(isStopRequested()) return;
            robotCentricDrive(leftFront,leftBack,rightFront,rightBack,gamepad1.left_trigger,gamepad1.right_trigger);
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //arm si extender
            if(currentGamepad2.a && !previousGamepad2.a && collectScoreController.currentStatus!= CollectScoreController.CollectScoreStatus.COLLECT && collectScoreController.currentStatus != CollectScoreController.CollectScoreStatus.WAIT_FOR_COLLECT)
            {
            collectScoreController.currentStatus = CollectScoreController.CollectScoreStatus.COLLECT;
            }
            else if(currentGamepad2.a && !previousGamepad2.a)
            {
                collectScoreController.currentStatus= CollectScoreController.CollectScoreStatus.INIT;
                collectScoreController.currentStatus= CollectScoreController.CollectScoreStatus.INIT;
            }

            if(currentGamepad2.b && !previousGamepad2.b)
            {
                if(armController.currentStatus!= ArmController.ArmStatus.HIGH) {
                    armController.currentStatus = ArmController.ArmStatus.HIGH;
                }
                else
                {
                    armController.currentStatus= ArmController.ArmStatus.INIT;
                }
            }

            //extender manual
            if(currentGamepad2.dpad_up)
            {
                extenderController.currentStatus= ExtenderController.ExtenderStatus.RUNTO;
                extenderTargetPosition=extenderController.extender.getCurrentPosition()+50;
                if(extenderTargetPosition>0) {
                    armController.currentStatus = ArmController.ArmStatus.RUNTO;
                    armTargetPosition = 500;
                }
                else if(extenderController.extender.getCurrentPosition()>300)
                {
                    armController.currentStatus = ArmController.ArmStatus.RUNTO;
                    armTargetPosition = 500;
                }
            }
            if(currentGamepad2.dpad_down)
            {
                extenderController.currentStatus= ExtenderController.ExtenderStatus.RUNTO;
                extenderTargetPosition=extenderController.extender.getCurrentPosition()-10;
                armController.currentStatus= ArmController.ArmStatus.RUNTO;
                armTargetPosition=armController.arm.getCurrentPosition()-10;
            }

            //activare intake
            if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper && intakeController.currentStatus!= IntakeController.IntakeStatus.COLLECT)
            {
                intakeController.currentStatus= IntakeController.IntakeStatus.COLLECT;
            }
            else if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
            {
                intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
            }

            if(currentGamepad2.left_trigger>0 && intakeController.currentStatus!= IntakeController.IntakeStatus.REVERSE)
            {
                intakeController.currentStatus= IntakeController.IntakeStatus.REVERSE;
            }
            else if(currentGamepad2.left_trigger>0 && intakeController.currentStatus == IntakeController.IntakeStatus.REVERSE)
            {
                intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
            }

            if(colorController.timer_nothing.seconds() > 0.5 || (currentGamepad2.x && !previousGamepad2.x))
            {
                clawController.currentStatus= ClawController.ClawStatus.OPEN;
            }
            else if(colorController.timer_yellow.seconds()>0.3 || colorController.timer_blue.seconds()>0.3 || colorController.timer_red.seconds()>0.3)
            {
                clawController.currentStatus = ClawController.ClawStatus.CLOSE;
            }
            clawPositionController.update();
            clawRotateController.update();
            clawController.update();
            armController.update(armTargetPosition);
            extenderController.update(extenderTargetPosition);
            colorController.update();
            fourbarController.update();
            intakeController.update();
            collectScoreController.update();
            telemetry.addData("arm powerdraw", armController.arm.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("arm power", armController.arm.getPower());
            telemetry.addData("arm status", armController.currentStatus);
            telemetry.addData("collect status", collectScoreController.currentStatus);
            telemetry.addData("timer interval", collectScoreController.timer_interval.seconds());
            telemetry.addData("extender status", extenderController.currentStatus);
            telemetry.addData("extender target", extenderController.extender.getTargetPosition());
            telemetry.addData("extender position", extenderController.extender.getCurrentPosition());
            telemetry.addData("claw", clawController.currentStatus);
            telemetry.addData("red", robot.colorSensor.red());
            telemetry.addData("green", robot.colorSensor.green());
            telemetry.addData("blue", robot.colorSensor.blue());
            telemetry.addData("color status", colorController.currentStatus);
            telemetry.update();
        }
    }
}