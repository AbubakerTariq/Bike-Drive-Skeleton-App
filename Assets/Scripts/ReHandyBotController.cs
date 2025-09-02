using Articares.Distal;
using DG.Tweening;
// using UnityEngine.UI;
// using UnityEngine.Video;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Timers;
//using System.Runtime.Remoting.Messaging;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.SceneManagement;

public class ReHandyBotController : MonoBehaviour
{
    ////////////////////////////////////////////////////////////////////////////
    // Real-time steps - CRITICAL:
    ////////////////////////////////////////////////////////////////////////////

    // Application time step:
    public const int DT_STEP_APP_MSEC = 25;

    ////////////////////////////////////////////////////////////////////////////
    // Game control modes - CRITICAL:
    //////////////////////////////////////////////////////////////////////////// 

    public const int CTRL_ASSISTED            = 1;
    public const int CTRL_AUTO_STEER_THROTTLE = 2;
    public const int CTRL_AUTO_STEER          = 3;
    public const int CTRL_MANUAL_SIMPLE       = 4;

    public const int CASE_CTRL_MODE = CTRL_AUTO_STEER_THROTTLE;

    ////////////////////////////////////////////////////////////////////////////
    // User-based game parameters - CRITICAL (30.08.2025):
    //////////////////////////////////////////////////////////////////////////// 

    public const float FACT_ASSIST_STEER    = 0.0f;  
    public const float FACT_ASSIST_THROTTLE = 1.0f;

    public const float FRAC_POS_ROT_INPUT_USER = 0.5f; // scaling factor for user's rotational inputs (based on user's rom, for example)

    ////////////////////////////////////////////////////////////////////////////
    // SetExercise() parameters - CRITICAL:
    //////////////////////////////////////////////////////////////////////////// 

    float OFFS_FORCE_RADIAL_INIT = 0f;
    float OFFS_TORQUE_ROT_INIT   = 0f;

    private bool SAFETY_TCP_APP_ON     = false;
    private bool STABILITY_SET_TARG_ON = true;  

    const bool ENGAGE_BRAKE    = false;
    const bool DISENGAGE_BRAKE = true;

    ////////////////////////////////////////////////////////////////////////////
    // Target indices:
    ////////////////////////////////////////////////////////////////////////////

    public const int NUM_TARGETS = 1;

    private byte IDX_TARG_BASE = 1;
    // private byte IDX_TARG_LIM = 2;

    ////////////////////////////////////////////////////////////////////////////
    // Object instances:
    ////////////////////////////////////////////////////////////////////////////

    public static ReHandyBotController instance;
    private DistalComm distalRobot = new(); // Distal Control Library object

    ////////////////////////////////////////////////////////////////////////////
    // Configuration values:
    ////////////////////////////////////////////////////////////////////////////

    private float FORCE_GAIN_RADIAL = 9f;
    private float FORCE_GAIN_ROT = 14f;

    private float K_STIFF_RADIAL_WALL = 2500f; // use with zero feedback gain
    private float B_DAMP_RADIAL_WALL  = 0f; // 40f; // rely on embedded HL_SetTarget stability

    private float K_STIFF_ROT_WALL = 1.2f; // use with zero feedback gain
    private float B_DAMP_ROT_WALL  = 0f; // 0.092f; // rely on embedded HL_SetTarget stability

    private float POS_RADIAL_MIN = 0.0145f;
    private float POS_RADIAL_MAX = 0.06f;  

    private float POS_ROT_MIN = -Mathf.PI / 2f;
    private float POS_ROT_MAX =  Mathf.PI / 2f;

    ////////////////////////////////////////////////////////////////////////////
    // RHB control settings - CRITICAL
    // NOTE: use [RHB ctrl params - stability v5b game settings 4-axis.xlsx] to calculate damping as a function of stiffness
    ////////////////////////////////////////////////////////////////////////////

    // Throttle - BASELINE haptics settings:
    [HideInInspector] public float POS_RADIAL_BASE_THROT = 0.029f;
    private float K_STIFF_RADIAL_BASE_THROT = 2500f;
    private float B_DAMP_RADIAL_BASE_THROT = 0f; // rely on embedded HL_SetTarget stability 

    // Steering - BASELINE haptics settings:
    [HideInInspector] public float POS_ROT_BASE_STEER = 0f;
    private float K_STIFF_ROT_BASE_STEER = 0.05f;  
    static float B_DAMP_ROT_BASE_STEER = 0f; // rely on embedded HL_SetTarget stability

    ////////////////////////////////////////////////////////////////////////////
    // Impedance for RHB motion limits:
    ////////////////////////////////////////////////////////////////////////////   

    private float K_STIFF_ROT_LIM = 0.6f;
    private float B_DAMP_ROT_LIM = 0f; // rely on embedded HL_SetTarget stability

    public float ANGLE_ROT_LIM_DEG = 45.0f;

    ////////////////////////////////////////////////////////////////////////////
    // Feedback control parameters:
    ////////////////////////////////////////////////////////////////////////////

    // Preview-ahead time - CRITICAL (26.08.2025):
    public float DT_PREVIEW = 2.0f; //  1.3f;

    // Gain for tracking reference roll angle - CRITICAL
    public float P_GAIN_ERR_POS_TARG = 0.06f; // 0.09f; // 0.045f; //
    // public float P_GAIN_ERR_POS_NEAR = 0.01f; // TODO: keep or discard

    // Gain(s) for tracking reference RHB rotation angle:
    public float P_GAIN_ANGLE_INPUT = 3.5f; // 3.5f;  
    public float D_GAIN_ANGLE_INPUT =  0f;

    const int SGN_ANGLE_CTRL = -1; // due to angle_ctrl sign convention in MotorbikeController

    // Stiffness for tracking control;
    float K_STIFF_TRACKING = 2.0f; // rely on embedded HL_SetTarget stability

    ////////////////////////////////////////////////////////////////////////////
    // Data structures from bike and track objects:
    ////////////////////////////////////////////////////////////////////////////

    static Vector3 NULL_VECTOR3 = Vector3.zero;
    const float NULL_VALUE      = 0f;
 
    //  Bike coordinates:
    private Vector3 pos_bike      = NULL_VECTOR3;
    private Vector3 dt_pos_bike   = NULL_VECTOR3;
    private Vector3 dir_unit_bike = NULL_VECTOR3;

    // Track coordinates:
    private Vector3 pos_ctrline_near   = NULL_VECTOR3;
    private Vector3 vect_ctrline_tang  = NULL_VECTOR3;
    private float curv_ctrline_near    = NULL_VALUE;
    private float ang_ctrline_tang     = NULL_VALUE;
    private float dist_ctrline_near    = NULL_VALUE; 

    // Bike pose:
    private float angle_roll_bike      = NULL_VALUE;
    private float dt_angle_roll_bike   = NULL_VALUE;
    private float angle_ctrl_signed    = NULL_VALUE;
    private float dt_angle_ctrl_signed = NULL_VALUE;

    // Feedback control:
    private float input_steer_targ = NULL_VALUE;

    ////////////////////////////////////////////////////////////////////////////
    // Loader:
    ////////////////////////////////////////////////////////////////////////////

    [Space] [Header("UI")]
    [SerializeField] private GameObject loader;
    [SerializeField] private GameObject exerciseGuidelineText;
    [SerializeField] private TMP_Text loaderText;

    ////////////////////////////////////////////////////////////////////////////
    // RHB info related variables:
    ////////////////////////////////////////////////////////////////////////////
    private bool RHBConnected => distalRobot.is_device_connected;
    public DistalComm.ExerciseData distal_data => distalRobot.DistalData;
    public bool ExerciseActive => isExerciseStarted;

    ////////////////////////////////////////////////////////////////////////////
    // Exercise-related variables:
    ////////////////////////////////////////////////////////////////////////////

    private bool isSystemStarted = false;
    public bool isExerciseStarted = false; // changed to public for access by DataManager (27.08.2025)
    private bool isExerciseStopping = false;

    ////////////////////////////////////////////////////////////////////////////
    // Constants:
    ////////////////////////////////////////////////////////////////////////////

    private const int MaxAttempts = 10;
    private const string ServerIP = "192.168.102.1";
    private const int ServerPort = 3002;

    ////////////////////////////////////////////////////////////////////////////
    // Misc. variables:
    ////////////////////////////////////////////////////////////////////////////

    private Queue<Action> MainThreadActionQueue = new();

    // Replaced by MotionRoutineRHBSimple() (22.08.2025):
    /*
    private Coroutine motionRoutineRadial;
    private Coroutine motionRoutineRotational;
    private bool isMoving = false;
    private bool isRotating = false;
    */

    private float pos_radial_min;

    private Thread connectionThread;
    private Tween connectionTween;

    private bool isCalibrated = false;
    private bool allowCalibration = false;

    public Action OnExerciseStart;
    public Action OnExerciseStop;

    private const string PrototypeSceneName = "Prototype";

    ////////////////////////////////////////////////////////////////////////////
    // Control loop timers:
    ////////////////////////////////////////////////////////////////////////////

    private bool timerActive = false;
    private bool timerActivePrev = false;
    private bool timerLocked = false;
    // private bool timerLockDetected = false;

    private float timeElapsedValue = 0f;

    public int step_count = 0;

    ////////////////////////////////////////////////////////////////////////////
    // Thread and timer for SetTarget process:
    ////////////////////////////////////////////////////////////////////////////

    private System.Timers.Timer timerSetTarget;
    private Thread threadTimerSetTarget;

    ////////////////////////////////////////////////////////////////////////////
    // Data display:
    ////////////////////////////////////////////////////////////////////////////

    private int  DT_DISP_DATA_MSEC      = 1000;
    private bool DISP_TIMER_ACTIVITY_ON = true;

    ////////////////////////////////////////////////////////////////////////////
    // Austo steer control struct:
    ////////////////////////////////////////////////////////////////////////////

    public struct FeedbackControl
    {
        public Vector3 pos_preview;
        public Vector3 pos_track_targ;
        public float angle_roll_targ;
        public float input_steer_targ;
        public float curv_ctrline_preview;
        public float sin_dev_targ;
        public Vector3 vect_ctrline_tang_target;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Public DATA VARIABLES for sharing data among classes:
    ////////////////////////////////////////////////////////////////////////////

    public FeedbackControl fbk_ctrl_data = new();

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // METHODS SECTION:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    // Basic methods:
    ////////////////////////////////////////////////////////////////////////////

    #region MonoBehavior Functions

    private void Awake()
    {
        // Singleton logic
        if (instance != null && instance != this)
        {
            Destroy(gameObject);
            return;
        }

        instance = this;
        DontDestroyOnLoad(gameObject);

        // Set log level so that the logs are stored in Nlog file
        distalRobot.SetLogLevel(DistalComm.DistalLogLevel.Info);
    }

    private void Start()
    {
        // Reset Ethernet port to prevent those frequent connection delays:
        System.Diagnostics.Process.Start("ethernet_reset.bat");

        // Start is only called once as this is a singleton object so we will only connect once at the beginning
        ConnectRHB();

        // Setup Set Target events process SetupSetTargetEvents() - removed 30.08.2025
    }

    private void OnApplicationQuit()
    {
        // StopSetTargetEvents() removed 30.08.2025

        // Stop all RHB related processes when the application is closed
        connectionThread?.Abort();
        connectionTween?.Kill();

        if (distalRobot == null || !RHBConnected)
            return;

        if (isExerciseStarted) distalRobot.StopExercise();
        if (isSystemStarted) distalRobot.StopSystem();
        if (RHBConnected) distalRobot.CloseConnection();

        distalRobot = null;
        instance = null;
    }
    #endregion

    ////////////////////////////////////////////////////////////////////////////
    // Timed update:
    ////////////////////////////////////////////////////////////////////////////

    private void Update()
    {
        ////////////////////////////////////////////////////////////////////////////
        // State check:
        ////////////////////////////////////////////////////////////////////////////

        // If robot is calibrated and user presses Enter, exercise state will be toggled
        // The exercise will start now if it hasn't started already
        if (isCalibrated && Input.GetKeyDown(KeyCode.Return))
            ToggleExerciseState();

        ////////////////////////////////////////////////////////////////////////////
        // Allow the user to press Y to calibrate the robot:
        ////////////////////////////////////////////////////////////////////////////

        if (allowCalibration && Input.GetKeyDown(KeyCode.Y))
            Calibrate(OnCalibrate);

        ////////////////////////////////////////////////////////////////////////////
        // Thread sleep & time elapsed computation:
        ////////////////////////////////////////////////////////////////////////////    

        // Conditional thread sleep:
        timerLocked = true;

        while (timerLocked)
        {
            System.Threading.Thread.Sleep(DT_STEP_APP_MSEC);
            timerLocked = false;
        }

        if (timerActive)
        {
            // Restart timer:
            if (timerActivePrev != timerActive)
                timeElapsedValue = 0f;
            else
                timeElapsedValue += Time.deltaTime;
        }

        // Record timer state for next step:
        timerActivePrev = timerActive;

        // Time elapsed computation:
        TimeSpan timeElapsedSpan = TimeSpan.FromSeconds(timeElapsedValue);

        ////////////////////////////////////////////////////////////////////////////
        // RHB coordinates:
        //////////////////////////////////////////////////////////////////////////// 

        float pos_radial = ReHandyBotController.instance.distal_data.PositionR;
        float pos_rot = ReHandyBotController.instance.distal_data.PositionP;

        ////////////////////////////////////////////////////////////////////////////
        // Retrieve data from bike and track objects:
        ////////////////////////////////////////////////////////////////////////////

        if (ExerciseActive && MotorbikeController.instance != null && Track.instance != null)
        {
            // Retrieve bike coordinates from MotorbikeController object:
            pos_bike      = MotorbikeController.instance.bike_coords_data.pos_bike;
            dt_pos_bike   = MotorbikeController.instance.bike_coords_data.dt_pos_bike;
            dir_unit_bike = MotorbikeController.instance.bike_coords_data.dir_unit_bike;

            // Retrieve track coordinates from MotorbikeController object:
            pos_ctrline_near  = MotorbikeController.instance.track_coords_data.pos_ctrline_near;
            vect_ctrline_tang = MotorbikeController.instance.track_coords_data.vect_ctrline_tang;
            curv_ctrline_near = MotorbikeController.instance.track_coords_data.curv_ctrline_near;
            ang_ctrline_tang  = MotorbikeController.instance.track_coords_data.ang_ctrline_tang;
            dist_ctrline_near = MotorbikeController.instance.track_coords_data.dist_ctrline_near;

            // Retrieve bike pose coordinates:
            angle_roll_bike      =                  MotorbikeController.instance.bike_pose_data.angle_roll_bike;
            dt_angle_roll_bike   =                  MotorbikeController.instance.bike_pose_data.dt_angle_roll_bike;
            angle_ctrl_signed    = SGN_ANGLE_CTRL * MotorbikeController.instance.bike_pose_data.angle_ctrl;
            dt_angle_ctrl_signed = SGN_ANGLE_CTRL * MotorbikeController.instance.bike_pose_data.dt_angle_ctrl;
        }

        ////////////////////////////////////////////////////////////////////////////
        // Compute steering input - feedback based 
        // Update public DATA VARIABLES for sharing among other classes (for atomicity & real-time updating):
        ////////////////////////////////////////////////////////////////////////////
        
        if (ExerciseActive && MotorbikeController.instance != null && Track.instance != null)
            input_steer_targ = SteerInputTarget(pos_bike, dt_pos_bike, dir_unit_bike, out fbk_ctrl_data);

        ////////////////////////////////////////////////////////////////////////////
        // Set Target commands for steering and throttle - CRITICAL:
        //////////////////////////////////////////////////////////////////////////// 

        switch (CASE_CTRL_MODE)
        {
            case CTRL_ASSISTED:

                CmdSetTargetCtrlFeedback(
                    FRAC_POS_ROT_INPUT_USER * angle_roll_bike, 
                    FACT_ASSIST_STEER * K_STIFF_TRACKING); // TODO: add baseline stiffness for zero assist
                break;

            case CTRL_AUTO_STEER_THROTTLE:
            case CTRL_AUTO_STEER:

                CmdSetTargetCtrlFeedback(
                    FRAC_POS_ROT_INPUT_USER * angle_roll_bike,
                    K_STIFF_TRACKING);
                break;

            case CTRL_MANUAL_SIMPLE:

                CmdSetTargetCtrlManualSimpleWithLimit(pos_rot);
                break;
        }

        ////////////////////////////////////////////////////////////////////////////
        // Update step counter:
        ////////////////////////////////////////////////////////////////////////////

        step_count++;

        ////////////////////////////////////////////////////////////////////////////
        // Using an action queue to perform Unity related tasks (i.e UI changes) which are not allowed to be done from a background thread
        ////////////////////////////////////////////////////////////////////////////

        while (MainThreadActionQueue.Count > 0)
            MainThreadActionQueue.Dequeue().Invoke();

        ////////////////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////////////////

        if (ExerciseActive && step_count % (DT_DISP_DATA_MSEC / DT_STEP_APP_MSEC) == 0 && DISP_TIMER_ACTIVITY_ON)
        {
            // Time elapsed display:
            string timeElapsedText = String.Format("{0:#00}", timeElapsedSpan.Minutes) + ":" + String.Format("{0:#00}", timeElapsedSpan.Seconds);

            ExternalConsoleLogger.Log("Update(" + step_count + ") t [" + String.Format("{0:#0.000}", timeElapsedValue) + "]:");
            ExternalConsoleLogger.Log("   pos_bike       " + pos_bike);
            ExternalConsoleLogger.Log("   pos_preview    " + fbk_ctrl_data.pos_preview);
            ExternalConsoleLogger.Log("   pos_track_targ " + fbk_ctrl_data.pos_track_targ);
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("   angle_ctrl, angle_roll_bike[" + String.Format("{0:#0.000}", angle_ctrl_signed) + "] [" + String.Format("{0:#0.000}", angle_roll_bike) + "]");
            // ExternalConsoleLogger.Log("   sgn*err_pos_targ      [" + String.Format("{0:#0.00}", sgn_turn_targ * err_pos_targ.magnitude) + "]");
            ExternalConsoleLogger.Log("   angle_roll_bike (ref, val) [" + String.Format("{0:#0.000}", fbk_ctrl_data.angle_roll_targ) + "] [" + String.Format("{0:#0.000}", angle_roll_bike) + "]");
            ExternalConsoleLogger.Log("   pos_rot (ref, val)    [" + String.Format("{0:#0.000}", input_steer_targ) + "] [" + String.Format("{0:#0.000}", pos_rot) + "]");
            ExternalConsoleLogger.Log(" ");
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Basic steering control:
    ////////////////////////////////////////////////////////////////////////////

    float SteerInputTarget(Vector3 pos_bike_this, Vector3 dt_pos_bike_this, Vector3 dir_unit_bike_this, out FeedbackControl fbk_ctrl) {

        float input_steer_targ_this = NULL_VALUE;

        Vector3 pos_preview = NULL_VECTOR3;
        Vector3 pos_track_targ = NULL_VECTOR3;
        float angle_roll_targ = NULL_VALUE;
        float curv_ctrline_preview = NULL_VALUE;
        float sin_dev_targ = NULL_VALUE; // angular deviation of bike's heading wrt to target
        Vector3 vect_ctrline_tang_target = NULL_VECTOR3;

        // Deviation wrt target point on track:
        Vector3 err_pos_targ = NULL_VECTOR3;
        Vector3 vect_unit_turn_targ = NULL_VECTOR3;
        float sgn_turn_targ = 0f;

        // Deviation wrt nearest point on track:
        Vector3 err_pos_near = NULL_VECTOR3;
        Vector3 vect_unit_turn_near = NULL_VECTOR3;

        Vector3 vect_bike_to_targ = NULL_VECTOR3;

        TrackDataFbkControl(
            pos_bike_this, dt_pos_bike_this, dir_unit_bike_this, 
            DT_PREVIEW, Track.instance, 
            ref pos_preview, ref pos_track_targ,  
            ref curv_ctrline_preview, ref vect_ctrline_tang_target);

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 2: lateral displacement control - roll angle target:
        ////////////////////////////////////////////////////////////////////////////

        // Deviation wrt target point on track:
        err_pos_targ = pos_track_targ - pos_preview;
        vect_unit_turn_targ = Vector3.Cross(dir_unit_bike_this, err_pos_targ.normalized); // test vector to establish turn direction
        sgn_turn_targ = (float) Math.Sign(-vect_unit_turn_targ.y);

        angle_roll_targ = P_GAIN_ERR_POS_TARG* sgn_turn_targ * err_pos_targ.magnitude;

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 3: steering input - KEY STEP
        ////////////////////////////////////////////////////////////////////////////

        input_steer_targ_this = P_GAIN_ANGLE_INPUT* (angle_roll_targ - angle_roll_bike); // - D_GAIN_ANGLE_INPUT * dt_angle_roll_bike; 

        ////////////////////////////////////////////////////////////////////////////
        // Feedback control 4: angular deviation of bike's heading wrt to target:
        ////////////////////////////////////////////////////////////////////////////  
        
        Vector3 vect_unit_dev_tangent = Vector3.Cross(
            vect_ctrline_tang.normalized, vect_ctrline_tang_target.normalized);

        sin_dev_targ = vect_unit_dev_tangent.y;

        ////////////////////////////////////////////////////////////////
        // Update data variables' struct for sharing among other classes (for atomicity & real-time updating):
        ////////////////////////////////////////////////////////////////    

        fbk_ctrl.pos_preview = pos_preview;
        fbk_ctrl.pos_track_targ = pos_track_targ;
        fbk_ctrl.angle_roll_targ = angle_roll_targ;
        fbk_ctrl.input_steer_targ = input_steer_targ_this;
        fbk_ctrl.curv_ctrline_preview = curv_ctrline_preview;
        fbk_ctrl.sin_dev_targ = sin_dev_targ;
        fbk_ctrl.vect_ctrline_tang_target = vect_ctrline_tang_target;

        return input_steer_targ_this;
    }

    #region Exercise tasks
    private void CmdSetTargetCtrlManualSimpleWithLimit(float pos_rot)
    {
        ////////////////////////////////////////////////////////////////////////////
        // Impedance parameters for rotation angle limit:
        ////////////////////////////////////////////////////////////////////////////

        // RADIAL parameters:
        float switch_radial = 1.0f;

        // ROTATIONAL parameters:
        float ANGLE_ROT_LIM = ANGLE_ROT_LIM_DEG * (float)Math.PI / 180f;

        float pos_eq_rot_curr;
        float k_rot_curr;
        float b_rot_curr;

        float switch_rot = 1.0f;

        ////////////////////////////////////////////////////////////////////////////
        // Compute impedance parameters:
        // If rotation limit is exceeded, apply limit values to rotational stiffness and damping:
        ////////////////////////////////////////////////////////////////////////////

        // NEW IMPLEMENTATION:
        // Compute equivalent stiffnes & equilibrium point for the combined steering stiffness and limit-position stiffness
        // Overcomes the limitation of RHB only allowing a single target (22.08.2025):

        float pos_eq_rot_lim_plus  =  ANGLE_ROT_LIM;
        float pos_eq_rot_lim_minus = -ANGLE_ROT_LIM;

        if (pos_rot > pos_eq_rot_lim_plus)
        {
            k_rot_curr = K_STIFF_ROT_BASE_STEER + K_STIFF_ROT_LIM;
            pos_eq_rot_curr = (K_STIFF_ROT_LIM * pos_eq_rot_lim_plus) / k_rot_curr;
        }

        else if (pos_rot < pos_eq_rot_lim_minus)
        {
            k_rot_curr = K_STIFF_ROT_BASE_STEER + K_STIFF_ROT_LIM;
            pos_eq_rot_curr = (K_STIFF_ROT_LIM * pos_eq_rot_lim_minus) / k_rot_curr;
        }
        else
        {
            k_rot_curr = K_STIFF_ROT_BASE_STEER;
            pos_eq_rot_curr = 0f;
        }

        // Assumes that damping is provided by embedded HL_SetTarget stability (22.08.2025):
        b_rot_curr = 0f;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        if (ExerciseActive)
            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_BASE_THROT, pos_eq_rot_curr,
                K_STIFF_RADIAL_BASE_THROT, k_rot_curr,
                B_DAMP_RADIAL_BASE_THROT, b_rot_curr,
                switch_radial, switch_rot);
        else
            success_set_target = false;
    }

    private void CmdSetTargetCtrlFeedback(float pos_eq_rot_ref, float k_rot_steer)
    {
        ////////////////////////////////////////////////////////////////////////////
        // Impedance parameters for rotation angle limit:
        ////////////////////////////////////////////////////////////////////////////

        // RADIAL parameters:
        float switch_radial = 1f;

        // ROTATIONAL parameters:
        float b_rot_steer = 0f; // Assumes that damping is provided by embedded HL_SetTarget stability
        float switch_rot = 1;

           ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        if (ExerciseActive)
            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_BASE_THROT, pos_eq_rot_ref,
                K_STIFF_RADIAL_BASE_THROT, k_rot_steer,
                B_DAMP_RADIAL_BASE_THROT, b_rot_steer,
                switch_radial, switch_rot);
        else
            success_set_target = false;        
    }
    #endregion

    ////////////////////////////////////////////////////////////////////////////
    // Data for feedback (assisted / auto-steer) control:
    ////////////////////////////////////////////////////////////////////////////
    
    private void TrackDataFbkControl(
        Vector3 pos_bike, Vector3 dt_pos_bike, Vector3 dir_unit_bike, 
        float dt_preview, Track track_this, 
        ref Vector3 pos_preview_this, ref Vector3 pos_track_targ_this, // ref Vector3 pos_track_near_this,
        ref float curv_ctrline_preview_this, ref Vector3 vect_ctrline_tang_target_this)
    {
        // Obtain preview point:
        float dist_preview = dt_pos_bike.magnitude * dt_preview; // distance to preview point ahead
        pos_preview_this = pos_bike + dist_preview * dir_unit_bike;

        // Obtain target point on track centerline:
        pos_track_targ_this = track_this.GetClosestPointOnCenterLine(pos_preview_this);

        // Obtain nearest point to bike on track centerline (TODO: keep or discard):
        // pos_track_near_this = track_this.GetClosestPointOnCenterLine(pos_bike);

        // Curvature of centerline at preview point:
        curv_ctrline_preview_this = track_this.GetCurvatureAtPosition(pos_track_targ_this);

        // Tangent vector at target point:
        vect_ctrline_tang_target_this = track_this.GetTangentAtPosition(pos_preview_this);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Ancillary functions - RHB control:
    ////////////////////////////////////////////////////////////////////////////

    #region RHB control functions

    private void ConnectRHB()
    {
        connectionTween?.Kill();
        connectionTween = DOVirtual.DelayedCall(10f, ReConnect);

        connectionThread?.Abort();
        connectionThread = new Thread(() =>
        {
            MainThreadActionQueue.Enqueue(() =>
            {
                loader.SetActive(true);
            });

            bool success = EstablishConnection();

            MainThreadActionQueue.Enqueue(() =>
            {
                connectionTween.Kill();

                if (success)
                {
                    loader.SetActive(false);
                    StartSystem(OnConnect);
                }
                else
                {
                    ReConnect();
                }
            });
        });
        connectionThread.Start();
    }

    private void ReConnect()
    {
        if (RHBConnected)
        {
            StartSystem(OnConnect);
            return;
        }
        ConnectRHB();
    }

    private void OnConnect()
    {
        SetBrakes(DISENGAGE_BRAKE, DISENGAGE_BRAKE);
        ExternalConsoleLogger.Log("        OnConnect(): SetBrakes(): cmd DISENGAGE \n");

        loader.SetActive(true);
        loaderText.text = "Align grippers horizontally and close the grippers\nCLICK on this screen and press Y to calibrate";
        allowCalibration = true;
    }

    private bool EstablishConnection(UnityAction onComplete = null)
    {
        if (RHBConnected)
        {
            onComplete?.Invoke();
            return true;
        }

        for (int i = 0; i < MaxAttempts; i++)
        {
            bool success = distalRobot.EstablishConnection(ServerIP, ServerPort);

            if (success)
            {
                onComplete?.Invoke();
                break;
            }
        }
        return RHBConnected;
    }

    private void StartSystem(UnityAction onComplete = null)
    {
        if (isSystemStarted)
        {
            distalRobot.SetSafety(SAFETY_TCP_APP_ON);
            onComplete?.Invoke();
            return;
        }

        for (int i = 0; i < MaxAttempts; i++)
        {
            bool success = distalRobot.StartSystem();

            if (success)
            {
                distalRobot.SetSafety(SAFETY_TCP_APP_ON);
                isSystemStarted = true;
                onComplete?.Invoke();
                break;
            }
        }
    }

    private void Calibrate(UnityAction onComplete = null)
    {
        for (int i = 0; i < MaxAttempts; i++)
        {
            if (distalRobot.Calibration(DistalComm.CalibrationType.AxisCalib)) break;
        }

        /*
        for (int i = 0; i < MaxAttempts; i++)
        {
            if (distalRobot.Calibration(DistalComm.CalibrationType.AllForceSensorsZeroCalib)) break;
        }
        */

        onComplete.Invoke();
    }

    private void OnCalibrate()
    {
        allowCalibration = false;

        StartExercise(ENGAGE_BRAKE, ENGAGE_BRAKE, () =>
        {
            DOVirtual.DelayedCall(0.1f, () =>
            {
                loader.SetActive(false);
                pos_radial_min = distal_data.PositionR;
                pos_radial_min = Math.Clamp(pos_radial_min, POS_RADIAL_MIN, POS_RADIAL_MAX);

                for (int i = 0; i < MaxAttempts; i++)
                {
                    bool success = distalRobot.StopExercise();

                    if (success)
                    {
                        SetBrakes(ENGAGE_BRAKE, ENGAGE_BRAKE);
                        ExternalConsoleLogger.Log("        OnCalibrate(): SetBrakes(): cmd ENGAGE \n");

                        isExerciseStarted = false;
                        isCalibrated = true;
                        SceneManager.LoadScene(PrototypeSceneName);
                        exerciseGuidelineText.SetActive(true);
                        break;
                    }
                }
            });
        });
    }

    private void ToggleExerciseState()
    {
        // Start exercise:
        if (!isExerciseStarted)
        {
            StartExercise(DISENGAGE_BRAKE, DISENGAGE_BRAKE, () =>
            {
                // Added this disengage command becausethe one in StartExercise() apparently has no effect (20.08.2025):
                SetBrakes(DISENGAGE_BRAKE, DISENGAGE_BRAKE);
                ExternalConsoleLogger.Log("        StartExercise(): SetBrakes(): before MotionRoutineRHBSimple - cmd DISENGAGE \n");

                bool success_all = MotionRoutineRadialRHBBaseline();

                ExternalConsoleLogger.Log("        --------------------------------------------------------------------");
                ExternalConsoleLogger.Log("        MotionRoutineRadialRHBBaseline() EXECUTED, success all [" + success_all + "] \n");
            });
        }

        // Stop exercise:
        else
            StopExercise();
    }

    private void StartExercise(bool unlockRadial, bool unlockRotational, UnityAction onComplete = null)
    {
        if (isExerciseStarted)
        {             
            // Removed 20.08.2025:
            // SetBrakes(unlockRadial, unlockRotational);
            // bool success_set_targ_empty = HL_SetTargetEmpty();

            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("....................................................................");
            ExternalConsoleLogger.Log("StartExercise(): isExerciseStarted  - return \n");

            onComplete?.Invoke();

            if (isCalibrated)
                OnExerciseStart?.Invoke();

            return;
        }

        bool startExerciseSucess = false;

        int i = 0;
        do
        {
            distalRobot.HL_StartExercise(
               NUM_TARGETS,
               unlockRadial, unlockRotational,
               OFFS_FORCE_RADIAL_INIT, OFFS_TORQUE_ROT_INIT,
               out bool startExerciseResponse, out bool setGainResponse,
               FORCE_GAIN_RADIAL, FORCE_GAIN_ROT, STABILITY_SET_TARG_ON);

            startExerciseSucess = startExerciseResponse;
        }
        while (++i <= MaxAttempts && !startExerciseSucess);
           
        if (startExerciseSucess) {
            isExerciseStarted = true;

            // Start timer:
            timerLocked = true;
            timerActivePrev = timerActive;
            timerActive = true;
            System.Threading.Thread.Sleep(DT_STEP_APP_MSEC);
            timerLocked = false;

            // Set force feedback gains:
            switch (CASE_CTRL_MODE)
            {
                case CTRL_ASSISTED:
                    SetGain(
                        (1f - FACT_ASSIST_THROTTLE) * FORCE_GAIN_RADIAL, 
                        (1f - FACT_ASSIST_STEER)    * FORCE_GAIN_ROT);
                    break;

                case CTRL_AUTO_STEER_THROTTLE:
                case CTRL_AUTO_STEER:

                    SetGain(0f, 0f);
                    break;

                case CTRL_MANUAL_SIMPLE:

                    SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
                    break;
            }

            // Initiate recording on new data file:
            DataManager.instance.SetupRecordingEvents();

            // Display section:
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("StartExercise(): SUCCESS timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");

            if (isCalibrated)
                OnExerciseStart?.Invoke();

            onComplete?.Invoke();

        } 
        else
        {
            // Display section:
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("StartExercise(): FAIL \n");
        }
    }

    private void StopExercise(UnityAction onComplete = null)
    {
        if (!isExerciseStarted)
        {
            // Removed 20.08.2025:
            // SetBrakes(ENGAGE_BRAKE, ENGAGE_BRAKE);  

            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("....................................................................");
            ExternalConsoleLogger.Log("StopExercise(): !isExerciseStarted  - return \n");

            OnExerciseStop?.Invoke();
            onComplete?.Invoke();
            return;
        }

        if (isExerciseStopping)
            return;

        isExerciseStopping = true;
        loaderText.text = "Stopping Exercise...";
        loader.SetActive(true);
        Time.timeScale = 0f;
        DOTween.PauseAll();

        // Stop timer:
        timerLocked = true;
        timerActivePrev = timerActive;
        timerActive = false;
        System.Threading.Thread.Sleep(DT_STEP_APP_MSEC);
        timerLocked = false;

        // Set default feedback gains:
        SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);

        // Replaced by MotionRoutineRHBSimple() (22.08.2025):
        /*
        if (isMoving)
        {
            isMoving = false;
            StopCoroutine(motionRoutineRadial);
        }

        if (isRotating)
        {
            isRotating = false;
            StopCoroutine(motionRoutineRotational);
        }
        */

        // Move RHB end effector to 'home' position (with minimum radial position);
        SetBrakes(DISENGAGE_BRAKE, DISENGAGE_BRAKE);
        ExternalConsoleLogger.Log("        StopExercise(): SetBrakes(): before MotionRoutineRHBSimple - cmd DISENGAGE \n");

        bool success_all = MotionRoutineRHBSimple(POS_RADIAL_MIN);

        ExternalConsoleLogger.Log("        --------------------------------------------------------------------");
        ExternalConsoleLogger.Log("        MotionRoutineRHBSimple() radial target [" + POS_RADIAL_MIN + "] EXECUTED, success all [" + success_all + "] \n");
    
        SetBrakes(ENGAGE_BRAKE, ENGAGE_BRAKE);
        ExternalConsoleLogger.Log("        StopExercise(): SetBrakes(): after MotionRoutineRHBSimple - cmd ENGAGE \n");

        isExerciseStarted = false;
        isExerciseStopping = false;

        // Set 'race started' flag (27.08.2025):
        DataManager.instance.isRaceStarted = false;

        OnExerciseStop?.Invoke();
        onComplete?.Invoke();
        loader.SetActive(false);
        Time.timeScale = 1f;
        DOTween.PlayAll();

        // Display section:
        ExternalConsoleLogger.Log(" ");
        ExternalConsoleLogger.Log("____________________________________________________________________");
        ExternalConsoleLogger.Log("StopExercise(): timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");

        // Replaced by MotionRoutineRHBSimple() (22.08.2025):
        /*
        motionRoutineRotational = StartCoroutine(MotionRoutineRotationalRHB(0f, () =>
        {
            motionRoutineRadial = StartCoroutine(MotionRoutineRadialRHB(POS_RADIAL_MIN, () =>
            {
        */
    }

    /// <summary>
    /// Sets ReHandyBot brakes, False = Engage, True = Disengage
    /// </summary>
    /// <param name="unlockRadial">Horizontal Axis</param>
    /// <param name="unlockRotational">Vertical Axis</param>
    private void SetBrakes(bool unlockRadial, bool unlockRotational, UnityAction onComplete = null)
    {
        bool success = false;

        for (int i = 0; i < MaxAttempts; i++)
        {
            success = distalRobot.ControlBrakes(unlockRadial, unlockRotational);

            if (success)
                break;
        }

        // Display section:
        /*
        if (success && unlockRadial)
            ExternalConsoleLogger.Log("        SetBrakes(): SUCCESS - DISENGAGE \n");
        else if (success && !unlockRadial)
            ExternalConsoleLogger.Log("        SetBrakes(): SUCCESS - ENGAGE \n");        
        else
            ExternalConsoleLogger.Log("        SetBrakes(): FAIL \n");
        */

        onComplete?.Invoke();
    }

    private bool SetTargetValidated(byte targetIndex,
        float radialValue, float rotationValue,
        float radialStiffness, float rotationStiffness,
        float radialDamping, float rotationDamping,
        float radialGain, float rotationGain, UnityAction onComplete = null)
    {
        const bool CHECK_EXERCISE_STATE = false;
        bool success = false;

        if (CHECK_EXERCISE_STATE && (!isExerciseStarted || isExerciseStopping))
            return success;

        radialValue = Mathf.Clamp(radialValue, POS_RADIAL_MIN, POS_RADIAL_MAX);
        rotationValue = Mathf.Clamp(rotationValue, POS_ROT_MIN, POS_ROT_MAX);

        for (int i = 0; i < MaxAttempts; i++)
        {
            success = distalRobot.HL_SetTarget(
                targetIndex,
                radialValue, rotationValue,
                radialStiffness, rotationStiffness,
                radialDamping, rotationDamping,
                radialGain, rotationGain);

            if (success)
            {
                onComplete?.Invoke();
                break;
            }
        }

        return success;
    }

    private bool HL_SetTargetEmpty(UnityAction onComplete = null)
    {
        return distalRobot.HL_SetTarget(
            IDX_TARG_BASE,
            POS_RADIAL_MIN, 0f,
            0f, 0f,
            0f, 0f,
            0f, 0f);
    }

    private void SetGain(float radialGain, float angularGain)
    {
        for (int i = 0; i < MaxAttempts; i++)
            if (distalRobot.SetGain(radialGain, angularGain)) 
                break;
    }

    public void SetOffsetForces(float radialOffsetForce, float angularOffsetForce)
    {
        for (int i = 0; i < MaxAttempts; i++)
            if (distalRobot.SetOffsetForces(radialOffsetForce, angularOffsetForce)) 
                break;
    }


    // Replaced with MotionRoutineRHBSimple() (22.08.2025):
    // private IEnumerator MotionRoutineRadialRHB(float target, UnityAction onComplete)

    // Replaced by MotionRoutineRHBSimple() (22.08.2025):
    // private IEnumerator MotionRoutineRotationalRHB(float target, UnityAction onComplete)

    private bool MotionRoutineRHBSimple(float pos_rad_targ)
    {
        const int DT_MOTION_ROUTINE_MSEC = 1500;
        const int N_STEPS_MOTION_ROUTINE = 60;

        bool success_all = false;

        float switch_var = 0f;

        for (int i = 0; i <= N_STEPS_MOTION_ROUTINE; i++)
        {

            // Note use of baseline impedance:
            bool success_step = SetTargetValidated(
                IDX_TARG_BASE,
                pos_rad_targ, 0f,
                K_STIFF_RADIAL_WALL, K_STIFF_ROT_WALL,
                B_DAMP_RADIAL_WALL, B_DAMP_ROT_WALL,
                switch_var, switch_var);

            if (success_all && !success_step)
                success_all = success_step;

            Thread.Sleep(DT_MOTION_ROUTINE_MSEC / N_STEPS_MOTION_ROUTINE);

            switch_var += 1f / N_STEPS_MOTION_ROUTINE;
        }

        return success_all;
    }

    private bool MotionRoutineRadialRHBBaseline()
    {
        const int DT_MOTION_BASE_MSEC = 1000;
        const int N_STEPS_MOTION_BASE = 40;

        bool success_all = true;

        float switch_var = 0f;

        for (int i = 0; i <= N_STEPS_MOTION_BASE; i++)
        {
            // Note use of baseline impedance:
            bool success_step = SetTargetValidated(
                IDX_TARG_BASE,
                POS_RADIAL_BASE_THROT, POS_ROT_BASE_STEER,
                K_STIFF_RADIAL_BASE_THROT, K_STIFF_ROT_BASE_STEER,
                B_DAMP_RADIAL_BASE_THROT, B_DAMP_ROT_BASE_STEER,
                switch_var, 1f);

            if (success_all && !success_step)
                success_all = success_step;

            Thread.Sleep(DT_MOTION_BASE_MSEC / N_STEPS_MOTION_BASE);

            switch_var += 1f / N_STEPS_MOTION_BASE;
        }

        /*
        ExternalConsoleLogger.Log(" ");
        ExternalConsoleLogger.Log("--------------------------------------------------------------------");
        ExternalConsoleLogger.Log("MotionRoutineRadialRHBBaseline() EXECUTED, success_all [" + success_all + "] \n");
        */

        return success_all;
    }

    #endregion

    // This is for usage for SetOffsetForces command, currently being called with dummy values
    private void SetOffsetForces()
    {
        ReHandyBotController.instance.SetOffsetForces(0f, 0f);
    }

    private void Destroy()
    {
        // StopSetTargetEvents() - removed 30.08.2025
    }

    // SetTarget functions removed 30.08.2025:
    /*
    private void SetupSetTargetEvents()
   
    private void StartSetTargetEvents()

    private void StopSetTargetEvents()

    private void SendCmdSetTargetSteerLimit(object sender, ElapsedEventArgs e)
    */

    ////////////////////////////////////////////////////////////////////////////
    // Ancillary functions:
    ////////////////////////////////////////////////////////////////////////////
    private Vector2 VectorXZ(Vector3 vect)
    {
        return new Vector2(vect.x, vect.x);
    }
}