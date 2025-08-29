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
    // Real-time steps (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // Application time step:
    public const int DT_STEP_APP_MSEC = 25;

    // Set Target command time step (TODO: keep or discard - together with StartSetTargetEvents()):
    public const int DT_STEP_SET_TARG_MSEC = 25;

    ////////////////////////////////////////////////////////////////////////////
    // SetExercise() parameters (CRITICAL):
    //////////////////////////////////////////////////////////////////////////// 
    
    public const bool AUTO_STEER_RHB_ON = true; // CRITICAL - use with care
    public const bool AUTO_THROTTLE_RHB_ON = true; // CRITICAL - use with care

    float OFFS_FORCE_RADIAL_INIT = 0f;
    float OFFS_TORQUE_ROT_INIT = 0f;

    private bool SAFETY_TCP_APP_ON = false;
    private bool STABILITY_SET_TARG_ON = true;  

    const bool ENGAGE_BRAKE = false;
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
    private float K_STIFF_ROT_BASE_STEER = 0.1f;  
    static float B_DAMP_ROT_BASE_STEER = 0f; // rely on embedded HL_SetTarget stability

    ////////////////////////////////////////////////////////////////////////////
    // Impedance for RHB motion limits:
    ////////////////////////////////////////////////////////////////////////////   

    private float K_STIFF_ROT_LIM = 0.6f;
    private float B_DAMP_ROT_LIM = 0f; // rely on embedded HL_SetTarget stability

    public float ANGLE_ROT_LIM_DEG = 45.0f;

    ////////////////////////////////////////////////////////////////////////////
    // Auto steer control parameters:
    ////////////////////////////////////////////////////////////////////////////
    
    // Tracking control input mode:
    const int INPUT_MODE_ANGLE_ROLL = 1;
    const int INPUT_MODE_ANGLE_CTRL = 2;

    const int CASE_INPUT_MODE = INPUT_MODE_ANGLE_ROLL;

    // CRITICAL: preview-ahead time (26.08.2025)
    public float DT_PREVIEW = 2.0f; //  1.3f;

    // Gain for tracking reference roll angle
    // CRITICAL: link it with FACTOR_STEER_DT_ANGLE_CTRL in MotorbikeController (26.08.2025):
    public float P_GAIN_ANGLE_INPUT_BIKE = 0.06f; // 0.09f; // 0.045f; //

    // Gain(s) for tracking reference RHB rotation angle:
    public float P_GAIN_POS_ROT_RHB = 3.5f; // 3.5f;  
    public float D_GAIN_POS_ROT_RHB =  0f;

    const int SGN_ANGLE_CTRL = -1; // due to angle_ctrl sign convention in MotorbikeController

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
    private float angle_roll           = NULL_VALUE;
    private float dt_angle_roll        = NULL_VALUE;
    private float angle_ctrl_signed    = NULL_VALUE;
    private float dt_angle_ctrl_signed = NULL_VALUE;

    // Preview tracking control:
    private Vector3 pos_preview        = NULL_VECTOR3;
    private Vector3 pos_track_targ     = NULL_VECTOR3;
    private float angle_input_ref      = NULL_VALUE;
    private float pos_rot_ref          = NULL_VALUE;
    private float curv_ctrline_preview = NULL_VALUE;
    private float sin_dev_targ         = NULL_VALUE; // angular deviation of bike's heading wrt to target
    private Vector3 vect_ctrline_tang_target = NULL_VECTOR3;

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

    public struct AutoSteerControl
    {
        public Vector3 pos_preview;
        public Vector3 pos_track_targ;
        public float angle_input_ref;
        public float pos_rot_ref;
        public float curv_ctrline_preview;
        public float sin_dev_targ;
        public Vector3 vect_ctrline_tang_target;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Public DATA VARIABLES for sharing data among classes:
    ////////////////////////////////////////////////////////////////////////////

    public AutoSteerControl auto_steer_ctrl_data = new();

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

        // Setup Set Target events process (TODO: keep or discard):
        // SetupSetTargetEvents();
    }

    private void OnApplicationQuit()
    {
        // Stop Set target events (TODO: keep or discard):
        // StopSetTargetEvents();

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
        // Extract data from bike and track objects:
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
            angle_roll           =                  MotorbikeController.instance.bike_pose_data.angle_roll;
            dt_angle_roll        =                  MotorbikeController.instance.bike_pose_data.dt_angle_roll;
            angle_ctrl_signed    = SGN_ANGLE_CTRL * MotorbikeController.instance.bike_pose_data.angle_ctrl;
            dt_angle_ctrl_signed = SGN_ANGLE_CTRL * MotorbikeController.instance.bike_pose_data.dt_angle_ctrl;
        }

        ////////////////////////////////////////////////////////////////////////////
        // Auto steer control:
        ////////////////////////////////////////////////////////////////////////////
        
        Vector3 err_pos_targ = NULL_VECTOR3;
        Vector3 vect_unit_turn_req = NULL_VECTOR3;
        float sgn_turn_req = 0f;

        Vector3 vect_bike_to_targ = NULL_VECTOR3;

        // Roll angle limit: TODO: remove at a later date
        // float ANGLE_ROLL_LOW = MotorbikeController.ANGLE_ROLL_LOW_DEG * (float)Math.PI / 180f;

        if (ExerciseActive && MotorbikeController.instance != null && Track.instance != null)
        {
            ////////////////////////////////////////////////////////////////////////////
            // Auto steer control 1: preview tracking - reference point on track:
            ////////////////////////////////////////////////////////////////////////////

            AutoSteerControlTrackData(pos_bike, dt_pos_bike, dir_unit_bike, DT_PREVIEW, Track.instance, 
                ref pos_preview, ref pos_track_targ, ref curv_ctrline_preview, ref vect_ctrline_tang_target);

            ////////////////////////////////////////////////////////////////////////////
            // Auto steer control 2: lateral displacement control - roll angle reference:
            ////////////////////////////////////////////////////////////////////////////

            // Required turn direction:
            err_pos_targ = pos_track_targ - pos_preview;
            vect_unit_turn_req = Vector3.Cross(dir_unit_bike, err_pos_targ.normalized); // test vector to establish turn direction
            sgn_turn_req = (float)Math.Sign(-vect_unit_turn_req.y);

            angle_input_ref = P_GAIN_ANGLE_INPUT_BIKE * sgn_turn_req * err_pos_targ.magnitude;

            // Roll angle limit: TODO: remove at a later date
            /*
            if (angle_input_ref > ANGLE_ROLL_LOW)
                angle_input_ref = ANGLE_ROLL_LOW;
            else if (angle_input_ref < -ANGLE_ROLL_LOW)
                angle_input_ref = -ANGLE_ROLL_LOW;
            */

            ////////////////////////////////////////////////////////////////////////////
            // Auto steer control 3: steering control - RHB rotation angle reference:
            ////////////////////////////////////////////////////////////////////////////

            if (CASE_INPUT_MODE == INPUT_MODE_ANGLE_ROLL)
                pos_rot_ref = P_GAIN_POS_ROT_RHB * (angle_input_ref - angle_roll); // - D_GAIN_POS_ROT_RHB * dt_angle_roll; 

            else if (CASE_INPUT_MODE == INPUT_MODE_ANGLE_CTRL)
                pos_rot_ref = P_GAIN_POS_ROT_RHB * (angle_input_ref - angle_ctrl_signed) - D_GAIN_POS_ROT_RHB * dt_angle_ctrl_signed;

            ////////////////////////////////////////////////////////////////////////////
            // Auto steer control 4: angular deviation of bike's heading wrt to target:
            ////////////////////////////////////////////////////////////////////////////   

            // vect_bike_to_targ = pos_track_targ - pos_bike;
            // Vector3 vect_unit_dev_targ = Vector3.Cross(dir_unit_bike, vect_bike_to_targ.normalized);

            // sin_dev_targ = vect_unit_dev_targ.y;

            Vector3 vect_unit_dev_tangent = Vector3.Cross(
                vect_ctrline_tang.normalized, vect_ctrline_tang_target.normalized);

            sin_dev_targ = vect_unit_dev_tangent.y;
        }

        ////////////////////////////////////////////////////////////////
        // Update public DATA VARIABLES for sharing among other classes (for atomicity & real-time updating):
        ////////////////////////////////////////////////////////////////    

        auto_steer_ctrl_data.pos_preview = pos_preview;
        auto_steer_ctrl_data.pos_track_targ = pos_track_targ;
        auto_steer_ctrl_data.angle_input_ref = angle_input_ref;
        auto_steer_ctrl_data.pos_rot_ref = pos_rot_ref;
        auto_steer_ctrl_data.curv_ctrline_preview = curv_ctrline_preview;
        auto_steer_ctrl_data.sin_dev_targ = sin_dev_targ;
        auto_steer_ctrl_data.vect_ctrline_tang_target = vect_ctrline_tang_target;

        ////////////////////////////////////////////////////////////////////////////
        // Command steer angle limits:
        ////////////////////////////////////////////////////////////////////////////

        if (AUTO_STEER_RHB_ON)
        {
            float SCALE_POS_ROT_REF = 1.0f; // 0.2f;
            // float pos_rot_ref_scaled = SCALE_POS_ROT_REF*auto_steer_ctrl_data.pos_rot_ref;
            float pos_rot_ref_scaled = SCALE_POS_ROT_REF* angle_roll;

            float k_rot_steer = 10.0f*K_STIFF_ROT_LIM;

            CmdSetTargetAutoSteer(pos_rot_ref_scaled, k_rot_steer);
        }
        else       
            CmdSetTargetSteerWithLimit(pos_rot);

        ////////////////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////////////////

        /*
        if (ExerciseActive && step_count % (DT_DISP_DATA_MSEC / DT_STEP_APP_MSEC) == 0 && DISP_TIMER_ACTIVITY_ON)
        {
            // Time elapsed display:
            string timeElapsedText = String.Format("{0:#00}", timeElapsedSpan.Minutes) + ":" + String.Format("{0:#00}", timeElapsedSpan.Seconds);

            ExternalConsoleLogger.Log("Update(" + step_count + ") t [" + String.Format("{0:#0.000}", timeElapsedValue) + "]:");
            ExternalConsoleLogger.Log("   pos bike " + pos_bike      );
            ExternalConsoleLogger.Log("   vel bike " + dt_pos_bike   );
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("   pos near  " + pos_ctrline_near  );
            ExternalConsoleLogger.Log("   vect tang " + vect_ctrline_tang );
            ExternalConsoleLogger.Log("   curvature [" + String.Format("{0:#0.000}", curv_ctrline_near) + "]");
            ExternalConsoleLogger.Log("   ang tang  [" + String.Format("{0:#0.00}", ang_ctrline_tang)   + "]");
            ExternalConsoleLogger.Log("   d ctrline [" + String.Format("{0:#0.00}", dist_ctrline_near)   + "]");
            ExternalConsoleLogger.Log(" ");
        }
        */

        if (ExerciseActive && step_count % (DT_DISP_DATA_MSEC / DT_STEP_APP_MSEC) == 0 && DISP_TIMER_ACTIVITY_ON)
        {
            // Time elapsed display:
            string timeElapsedText = String.Format("{0:#00}", timeElapsedSpan.Minutes) + ":" + String.Format("{0:#00}", timeElapsedSpan.Seconds);

            ExternalConsoleLogger.Log("Update(" + step_count + ") t [" + String.Format("{0:#0.000}", timeElapsedValue) + "]:");
            ExternalConsoleLogger.Log("   pos_bike       " + pos_bike);
            ExternalConsoleLogger.Log("   pos_preview    " + pos_preview);
            ExternalConsoleLogger.Log("   pos_track_targ " + pos_track_targ);
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("   angle_ctrl, angle_roll[" + String.Format("{0:#0.000}", angle_ctrl_signed) + "] [" + String.Format("{0:#0.000}", angle_roll) + "]");
            ExternalConsoleLogger.Log("   sgn*err_pos_targ      [" + String.Format("{0:#0.00}", sgn_turn_req*err_pos_targ.magnitude) + "]");
            ExternalConsoleLogger.Log("   angle_roll (ref, val) [" + String.Format("{0:#0.000}", angle_input_ref) + "] [" + String.Format("{0:#0.000}", angle_roll) + "]");
            ExternalConsoleLogger.Log("   pos_rot (ref, val)    [" + String.Format("{0:#0.000}", pos_rot_ref)    + "] [" + String.Format("{0:#0.000}", pos_rot)    + "]");
            ExternalConsoleLogger.Log(" ");
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
    }

    ////////////////////////////////////////////////////////////////////////////
    // Basic steering control:
    ////////////////////////////////////////////////////////////////////////////

    #region Exercise tasks
    private void CmdSetTargetSteerWithLimit(float pos_rot)
    {
        ////////////////////////////////////////////////////////////////////////////
        // Impedance parameters for rotation angle limit:
        ////////////////////////////////////////////////////////////////////////////

        // RADIAL parameters:
        float gain_radial = 1.0f;

        // ROTATIONAL parameters:
        float ANGLE_ROT_LIM = ANGLE_ROT_LIM_DEG * (float)Math.PI / 180f;

        float pos_eq_rot_curr;
        float k_rot_curr;
        float b_rot_curr;

        float gain_rot = 1.0f;

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
                gain_radial, gain_rot);
        else
            success_set_target = false;

        ////////////////////////////////////////////////////////////////////////////
        // Display section: 
        ////////////////////////////////////////////////////////////////////////////

        /*
        bool DISP_SET_TARG_ON = true;

        if (ExerciseActive && step_count % (DT_DISP_DATA_MSEC / DT_STEP_APP_MSEC) == 0 && DISP_SET_TARG_ON)
        {
            string str_timer = "[" + step_count + "]  t [" + String.Format("{0:#0.000}", timeElapsedValue) + "]  HL_SetTarget(): ";

            ExternalConsoleLogger.Log("____________________________________________________________________");
            if (success_set_target)
                ExternalConsoleLogger.Log(str_timer + "success");
            else
                ExternalConsoleLogger.Log(str_timer + "FAIL");

            ExternalConsoleLogger.Log(
                "pos:   RAD [" + String.Format("{0:#0.000}", pos_radial) + "]  ROT [" + String.Format("{0:#0.00}", pos_phi) +
                    "] (limit [" + String.Format("{0:#0.00}", ANGLE_ROT_LIM) + "]) \n" +
                "stiff: RAD [" + String.Format("{0:#0.0}", K_STIFF_RADIAL_BASE_THROT) + "]  ROT [" + String.Format("{0:#0.000}", k_rot_curr) + "] \n" +
                "damp:  RAD [" + String.Format("{0:#0.0}", B_DAMP_RADIAL_BASE_THROT) + "]  ROT [" + String.Format("{0:#0.000}", b_rot_curr) + "] \n");
        }
        */
    }

    private void CmdSetTargetAutoSteer(float pos_eq_rot_ref, float k_rot_steer)
    {
        ////////////////////////////////////////////////////////////////////////////
        // Impedance parameters for rotation angle limit:
        ////////////////////////////////////////////////////////////////////////////

        // RADIAL parameters:
        float gain_radial = 1.0f;

        // ROTATIONAL parameters:
        float b_rot_steer = 0f; // Assumes that damping is provided by embedded HL_SetTarget stability

        float gain_rot = 1.0f;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        if (ExerciseActive)
            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_BASE_THROT, pos_eq_rot_ref,
                K_STIFF_RADIAL_BASE_THROT, k_rot_steer,
                B_DAMP_RADIAL_BASE_THROT, b_rot_steer,
                gain_radial, gain_rot);
        else
            success_set_target = false;        
    }
    #endregion

    ////////////////////////////////////////////////////////////////////////////
    // Auto steer control:
    ////////////////////////////////////////////////////////////////////////////

    private void AutoSteerControlTrackData(Vector3 pos_bike, Vector3 dt_pos_bike, Vector3 dir_unit_bike, float dt_preview, 
        Track track_this, ref Vector3 pos_preview_this, ref Vector3 pos_track_targ_this, 
        ref float curv_ctrline_preview_this, ref Vector3 vect_ctrline_tang_target_this)
    {
        // Obtain preview point:
        float dist_preview = dt_pos_bike.magnitude * dt_preview; // distance to preview point ahead
        pos_preview_this = pos_bike + dist_preview * dir_unit_bike;

        // Obtain target point on track:
        pos_track_targ_this = track_this.GetClosestPointOnCenterLine(pos_preview_this);

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

        // for (int i = 0; i < MaxAttempts; i++)
        // {
        //     if (distalRobot.Calibration(DistalComm.CalibrationType.AllForceSensorsZeroCalib)) break;
        // }

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

            // Set feedback gains:
            if (AUTO_STEER_RHB_ON)
                SetGain(0f, 0f);
            else
                SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);

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
                for (int i = 0; i < MaxAttempts; i++)
                {
                    if (distalRobot.StopExercise())
                        break;

                    if (distalRobot.LastErrorMessage.Contains("Timeout while waiting for StopExercise response"))
                        continue;
                }

                // HL_SetTargetEmpty();

                isExerciseStarted = false;
                isExerciseStopping = false;
                SetBrakes(BRAKE_ENGAGE, BRAKE_ENGAGE);
                OnExerciseStop?.Invoke();
                onComplete?.Invoke();
                loader.SetActive(false);
                Time.timeScale = 1f;
                DOTween.PlayAll();
            }));
        }));
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

    ////////////////////////////////////////////////////////////////////////////
    // Replaced with MotionRoutineRHBSimple() (22.08.2025):
    ////////////////////////////////////////////////////////////////////////////

    /*
    private IEnumerator MotionRoutineRadialRHB(float target, UnityAction onComplete)
    {
        isMoving = true;
        SetGain(0f, 0f);

        System.Diagnostics.Stopwatch stopwatch = new();
        stopwatch.Start();

        float init_position = distal_data.PositionR;
        float current_target = init_position;
        float current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;
        float init_time_ms = current_time_ms;
        float prev_time_ms = current_time_ms;
        float speed_factor = 1f;

        while ((init_position < target && current_target < target) || (init_position >= target && current_target > target))
        {
            current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;

            if ((current_time_ms - prev_time_ms) >= DT_STEP_APP_MSEC)
            {
                if (prev_time_ms == 0)
                {
                    prev_time_ms = current_time_ms;
                    continue;
                }

                float t = (current_time_ms - init_time_ms) / 1000f * speed_factor;
                current_target = init_position + (target - init_position) * (10f * Mathf.Pow(t, 3f) - 15f * Mathf.Pow(t, 4f) + 6f * Mathf.Pow(t, 5f));

                // Check if current_target is overshooting actual target
                if (((init_position < target) && (current_target > target)) || ((init_position >= target) && (current_target < target)))
                    current_target = target;

                // Set Updated Target:
                current_target = Mathf.Clamp(current_target, POS_RADIAL_MIN, POS_RADIAL_MAX);

                distalRobot.HL_SetTarget(
                    IDX_TARG_BASE, 
                    current_target, 0f, 
                    K_STIFF_RADIAL_WALL, K_STIFF_ROT_WALL, 
                    B_DAMP_RADIAL_WALL, B_DAMP_ROT_WALL, 
                    1f, 1f);

                prev_time_ms = current_time_ms;
            }
            yield return null;
        } // end while

        if (!isExerciseStopping)
            distalRobot.HL_SetTarget(
                IDX_TARG_BASE, 
                target, 0f, 
                K_STIFF_RADIAL_WALL, 0f, 
                B_DAMP_RADIAL_WALL, 0f, 
                1f, 1f);

        stopwatch.Stop();
        // SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
        isMoving = false;
        onComplete?.Invoke();
    }
    */

    ////////////////////////////////////////////////////////////////////////////
    // Replaced by MotionRoutineRHBSimple() (22.08.2025):
    ////////////////////////////////////////////////////////////////////////////
    
    /*
    private IEnumerator MotionRoutineRotationalRHB(float target, UnityAction onComplete)
    {
        isRotating = true;
        SetGain(0f, 0f);

        System.Diagnostics.Stopwatch stopwatch = new();
        stopwatch.Start();

        float pos_phi_init = distal_data.PositionP;
        float current_target = pos_phi_init;
        float current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;
        float init_time_ms = current_time_ms;
        float prev_time_ms = current_time_ms;
        float speed_factor = 0.75f;

        while (((pos_phi_init < target) && (current_target < target)) || ((pos_phi_init >= target) && (current_target > target)))
        {
            current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;

            if ((current_time_ms - prev_time_ms) >= DT_STEP_APP_MSEC)
            {
                if (prev_time_ms == 0)
                {
                    prev_time_ms = current_time_ms;
                    continue;
                }

                float t = (current_time_ms - init_time_ms) / 1000f * speed_factor;
                current_target = pos_phi_init + (target - pos_phi_init) * (10f * Mathf.Pow(t, 3f) - 15f * Mathf.Pow(t, 4f) + 6f * Mathf.Pow(t, 5f));

                // Check if current_target is overshooting actual target
                if (((pos_phi_init < target) && (current_target > target)) || ((pos_phi_init >= target) && (current_target < target)))
                    current_target = target;

                // Set Updated Target
                current_target = Mathf.Clamp(current_target, -Mathf.PI / 2f, Mathf.PI / 2f);

                distalRobot.HL_SetTarget(
                    IDX_TARG_BASE, 
                    distal_data.PositionR, current_target,
                    K_STIFF_RADIAL_WALL, K_STIFF_ROT_WALL,
                    B_DAMP_RADIAL_WALL, B_DAMP_ROT_WALL,
                    1, 1);

                prev_time_ms = current_time_ms;
            }

            yield return null;
        }

        if (!isExerciseStopping) 
            distalRobot.HL_SetTarget(
                IDX_TARG_BASE, 
                distal_data.PositionR, target,
                K_STIFF_RADIAL_WALL, K_STIFF_ROT_WALL,
                B_DAMP_RADIAL_WALL, B_DAMP_ROT_WALL,
                1, 1);

        stopwatch.Stop();
        // SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
        isRotating = false;
        onComplete?.Invoke();
    }
    */

    private bool MotionRoutineRHBSimple(float pos_rad_targ)
    {
        const int DT_MOTION_ROUTINE_MSEC = 1500;
        const int N_STEPS_MOTION_ROUTINE = 60;

        bool success_all = false;

        float gain_var = 0f;

        for (int i = 0; i <= N_STEPS_MOTION_ROUTINE; i++)
        {

            // Note use of baseline impedance:
            bool success_step = SetTargetValidated(
                IDX_TARG_BASE,
                pos_rad_targ, 0f,
                K_STIFF_RADIAL_WALL, K_STIFF_ROT_WALL,
                B_DAMP_RADIAL_WALL, B_DAMP_ROT_WALL,
                gain_var, gain_var);

            if (success_all && !success_step)
                success_all = success_step;

            Thread.Sleep(DT_MOTION_ROUTINE_MSEC / N_STEPS_MOTION_ROUTINE);

            gain_var += 1f / N_STEPS_MOTION_ROUTINE;
        }

        return success_all;
    }

    private bool MotionRoutineRadialRHBBaseline()
    {
        const int DT_MOTION_BASE_MSEC = 1000;
        const int N_STEPS_MOTION_BASE = 40;

        bool success_all = true;

        float gain_var = 0f;

        for (int i = 0; i <= N_STEPS_MOTION_BASE; i++)
        {
            // Note use of baseline impedance:
            bool success_step = SetTargetValidated(
                IDX_TARG_BASE,
                POS_RADIAL_BASE_THROT, POS_ROT_BASE_STEER,
                K_STIFF_RADIAL_BASE_THROT, K_STIFF_ROT_BASE_STEER,
                B_DAMP_RADIAL_BASE_THROT, B_DAMP_ROT_BASE_STEER,
                gain_var, 1f);

            if (success_all && !success_step)
                success_all = success_step;

            Thread.Sleep(DT_MOTION_BASE_MSEC / N_STEPS_MOTION_BASE);

            gain_var += 1f / N_STEPS_MOTION_BASE;
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
        StopSetTargetEvents();
    }
    
    #region Set Target functions
    private void SetupSetTargetEvents()
    {
        ReHandyBotController.instance.OnExerciseStart += StartSetTargetEvents;
        ReHandyBotController.instance.OnExerciseStop += StopSetTargetEvents;
    }

    private void StartSetTargetEvents()
    {
        StopSetTargetEvents();

        threadTimerSetTarget = new Thread(() =>
        {
            timerSetTarget = new System.Timers.Timer(DT_STEP_SET_TARG_MSEC);
            timerSetTarget.Elapsed += SendCmdSetTargetSteerLimit;
            timerSetTarget.AutoReset = true;
            timerSetTarget.Start();
        });
        threadTimerSetTarget.Start();
    }

    private void StopSetTargetEvents()
    {
        threadTimerSetTarget?.Abort();
        timerSetTarget?.Stop();
        timerSetTarget?.Dispose();
    }

    private void SendCmdSetTargetSteerLimit(object sender, ElapsedEventArgs e)
    {
        // RHB coordinates:
        float pos_rot = ReHandyBotController.instance.distal_data.PositionP;

        CmdSetTargetSteerWithLimit(pos_rot);
    }   


    private Vector2 VectorXZ(Vector3 vect)
    {
        return new Vector2(vect.x, vect.x);
    }
    #endregion
}