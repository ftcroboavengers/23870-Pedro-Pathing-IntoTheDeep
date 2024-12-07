package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Disabled
@Autonomous(name = "MI States Basket Auton", group = "Robot")
public class RoboAvengersMIStatesAuton extends OpMode
{
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotor  sliderMotor      = null;
    public Servo    claw             = null; //the claw servo
    public Servo    clawHead         = null;

    private final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    private final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    private final double ARM_SCORE_HIGH_BASKET     = 100 * ARM_TICKS_PER_DEGREE;
    private final double SLIDER_TICKS_PER_MM = 537.7 / 120.0;
    private final double SLIDER_SCORING_IN_HIGH_BASKET = 475 * SLIDER_TICKS_PER_MM;
    private final double SLIDER_SAMPLE1_PICKUP = 200 * SLIDER_TICKS_PER_MM;
    private final double SLIDER_SAMPLE1_SCORE = 200 * SLIDER_TICKS_PER_MM;
    private final double SLIDER_BACK2_ZERO = 475 * SLIDER_TICKS_PER_MM;
    private final double CLAW_CLOSED = 0.0;
    private final double CLAW_OPEN   = 1.0;
    private int loopCounter = 0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState = 0; //variable to store the state of our auto
    private PathChain scorePreload, pickSample1Path, scoreSample1path, safeParkPath;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * [RAC] Our robot is 15.25 by 17.5 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8.75, 79.625, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose1 = new Pose(8.75, 130, Math.toRadians(90));

    // Back away from bucket after scorePose1
    private final Pose safeParkPose = new Pose (8.75, 108, Math.toRadians(90));

    // Pickup pose of our robot to collect the first sample (right)
    private final Pose pickSample1Pose1 = new Pose (8.75,112, Math.toRadians(90));

    // Pickup pose of our robot to collect the first sample (right)
    private final Pose pickSample1Pose2 = new Pose (46,112, Math.toRadians(90));

    // Back away from bucket after scorePose2
    private final Pose scorePose2 = new Pose (12, 132, Math.toRadians(135));


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init()
    {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initiate drive motors
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.3);

        // Initiate arm motor and claw servo
        armMotor        = hardwareMap.dcMotor.get("left_arm");
        sliderMotor     = hardwareMap.dcMotor.get("liftMotor");
        claw            = hardwareMap.servo.get("claw");
        clawHead        = hardwareMap.servo.get("clawHead");

        //define behavior of motors
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setTargetPosition(0);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setTargetPosition(0);

        // Build robot auton paths according to Pedro Pathing coordinates
        buildPaths();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop()
    {
        if(pathState == -1)
        {
            requestOpModeStop();
        }
        else
        {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            // These loop the movements of the robot
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
            telemetry.addData("Path state: ", pathState);
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.addData("heading: ", follower.getPose().getHeading());
            telemetry.addData("Arm Position: ", armMotor.getCurrentPosition());
            telemetry.addData("Slider Position: ", sliderMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start()
    {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop()
    {
        armMotor.setPower(0);
        sliderMotor.setPower(0);
    }

    /** This builds the path required for our robot to move and collect samples **/
    private void buildPaths()
    {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build();

        pickSample1Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickSample1Pose1)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickSample1Pose1.getHeading())
                .addPath(new BezierLine(new Point(pickSample1Pose1), new Point(pickSample1Pose2)))
                .setLinearHeadingInterpolation(pickSample1Pose1.getHeading(), pickSample1Pose2.getHeading())
                .build();

        scoreSample1path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickSample1Pose2), new Point(scorePose2)))
                .setLinearHeadingInterpolation(pickSample1Pose2.getHeading(), scorePose2.getHeading())
                .build();

        safeParkPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(safeParkPose)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), safeParkPose.getHeading())
                .build();
    }

    /** Runs the auton path **/
    private void autonomousPathUpdate()
    {
        switch (pathState)
        {
            case 0:
                claw.setPosition(CLAW_CLOSED);
                actionTimer.resetTimer();
                while (actionTimer.getElapsedTime() < 500)
                {
                    telemetry.addData("Step 1: Claw close: ", "Complete");
                    telemetry.addData("Step 1: Robot current x position", follower.getPose().getX());
                    telemetry.addData("Step 1: Robot current y position", follower.getPose().getY());
                    telemetry.addData("Step 1: Robot current heading", follower.getPose().getHeading());
                    telemetry.update();
                }
                setPathState(1);
                break;
            case 1:
//                armMotor.setTargetPosition((int) ARM_SCORE_HIGH_BASKET);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                armMotor.setPower(0.7);
//
//                sliderMotor.setTargetPosition((int) SLIDER_SCORING_IN_HIGH_BASKET);
//                sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                sliderMotor.setPower(0.7);
//
//                while ( armMotor.isBusy() || sliderMotor.isBusy() )
//                {
//                    telemetry.addData("Step 2: Robot arm ready for the top scoring basket: ", "InProgress");
//                    telemetry.addData("Step 2: Robot current x position", follower.getPose().getX());
//                    telemetry.addData("Step 2: Robot current y position", follower.getPose().getY());
//                    telemetry.addData("Step 2: Robot current heading", follower.getPose().getHeading());
//                    telemetry.update();
//                }
                setPathState(2);
                break;
            case 2:
//                if( (armMotor.getCurrentPosition() >= (int)ARM_SCORE_HIGH_BASKET
//                        || armMotor.getCurrentPosition() <= (int)ARM_SCORE_HIGH_BASKET - 2)
//                        && armMotor.isBusy() == false )
//                {
                    telemetry.addData("Step 3: Robot sample preload run to basket: ", "Complete");
                    telemetry.addData("Step 3: Robot current x position", follower.getPose().getX());
                    telemetry.addData("Step 3: Robot current y position", follower.getPose().getY());
                    telemetry.addData("Step 3: Robot current heading", follower.getPose().getHeading());
                    follower.followPath(scorePreload, true);
                    setPathState(3);
//                }
                break;
            case 3:
                // Step 4 Sample drop in top basket
                if( follower.getPose().getX() > (scorePose1.getX() - 1.0)
                        && follower.getPose().getY() > (scorePose1.getY() - 1.0))
                {
//                    clawHead.setPosition(0.7);
//                    telemetry.addData("Step 4A: Claw 0.7 rotate: ", "Complete ");
//                    telemetry.update();
//                    claw.setPosition(CLAW_OPEN);
//
//                    actionTimer.resetTimer();
//                    while (actionTimer.getElapsedTime() < 500) //increase it to 500 if needed
//                    {
//                        telemetry.addData("Step 4B: Preload Sample drop: ", "Complete");
//                    }
//                    clawHead.setPosition(0.25);
//
//                    actionTimer.resetTimer();
//                    while (actionTimer.getElapsedTime() < 250)
//                    {
//                        telemetry.addData("Step 4C: Claw head rotated 0.25", "Complete");
//                        telemetry.update();
//                    }
                    setPathState(4);
                }
                break;

            case 4:
                // Step 5 Strafe backwards to sample 1 pickup
                telemetry.addData("Step 5: Robot strafe to sample 1 pickup: ", "Complete");
                telemetry.addData("Step 5: Robot current x position", follower.getPose().getX());
                telemetry.addData("Step 5: Robot current y position", follower.getPose().getY());
                telemetry.addData("Step 5: Robot current heading", follower.getPose().getHeading());
                follower.followPath(pickSample1Path, true);
                setPathState(5);
                break;
            case 5:
                //Step 5 Retract arm for sample 1 collection
                if( follower.getPose().getX() > (pickSample1Pose2.getX() - 1.0)
                        && follower.getPose().getY() < (pickSample1Pose2.getY() + 1.0) )
                {
//                    sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                    sliderMotor.setTargetPosition((int)SLIDER_SAMPLE1_PICKUP);
//                    sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    sliderMotor.setPower(0.6);
//
//                    actionTimer.resetTimer();
//                    while ( sliderMotor.isBusy() || actionTimer.getElapsedTime() < 500 )
//                    {
//                        telemetry.addData("Step 6B: Robot slider sample 1 position", "InProgress");
//                        telemetry.update();
//                    }
//
//                    armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
//                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    armMotor.setPower(0.6);
//
//                    actionTimer.resetTimer();
//                    while ( armMotor.isBusy() || actionTimer.getElapsedTime() < 500 )
//                    {
//                        telemetry.addData("Step 6C: Robot arm sample1 pickup position", "InProgress");
//                        telemetry.update();
//                    }
                    setPathState(-1);
                }
                break;
        }
    }

    /** Identifies the current state of the path **/
    private void setPathState(int pState)
    {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
