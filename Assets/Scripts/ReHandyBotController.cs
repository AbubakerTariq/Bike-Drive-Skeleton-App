using Articares.Distal;
using DG.Tweening;
// using UnityEngine.UI;
// using UnityEngine.Video;
using System;
using System.Collections.Generic;
using System.Threading;
//using System.Runtime.Remoting.Messaging;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.SceneManagement;

public class ReHandyBotController : MonoBehaviour
{
    float FACT_DEG_2_RAD = (float)Math.PI / 180f;

    ////////////////////////////////////////////////////////////////////////////
    // Real-time steps - CRITICAL:
    ////////////////////////////////////////////////////////////////////////////

    // Application time step:
    public const int DT_STEP_APP_MSEC = 25;

    ////////////////////////////////////////////////////////////////////////////
    // Game control modes:
    //////////////////////////////////////////////////////////////////////////// 

    const int NULL_SETTING = -1;

    // public const bool USE_CONSTRAINED_STEER = false; // TODO: keep or discard

    public const int CTRL_ASSISTED                = 1;
    public const int CTRL_AUTO_STEER_AUTO_THROT   = 2;
    public const int CTRL_AUTO_STEER_MANUAL_THROT = 3;
    public const int CTRL_MANUAL_SIMPLE           = 4;

    public int CASE_CTRL_MODE = CTRL_MANUAL_SIMPLE; // DEFAULT setting before selection

    ////////////////////////////////////////////////////////////////////////////
    // User-based game parameters - CRITICAL (30.08.2025):
    //////////////////////////////////////////////////////////////////////////// 

    public float FACT_ASSIST_STEER       = 0f; // DEFAULT setting before selection
    public float FACT_ASSIST_THROTTLE    = 0f; // DEFAULT setting before selection

    // Scaling factor for user's rotational inputs (based on user's ROM, for example)
    public float FRAC_POS_ROT_INPUT_USER = 0.5f;  // DEFAULT setting before selection

    ////////////////////////////////////////////////////////////////////////////
    // Pre-game procedures stated - CRITICAL:
    //////////////////////////////////////////////////////////////////////////// 

    private const int ST_SET_CTRL_MODE            = 1;
    private const int ST_SET_FACT_ASSIST_STEER    = 2;
    private const int ST_SET_FACT_ASSIST_THROTTLE = 3;
    private const int ST_CALIBRATE                = 4;

    private int STATE_PREGAME = ST_SET_CTRL_MODE; // initial state

    ////////////////////////////////////////////////////////////////////////////
    // SetExercise() parameters:
    //////////////////////////////////////////////////////////////////////////// 

    private float OFFS_FORCE_RADIAL_INIT = 0f;
    private float OFFS_TORQUE_ROT_INIT   = 0f;

    private bool SAFETY_TCP_APP_ON       = false;
    private bool STABILITY_SET_TARG_ON   = true;  

    private const bool ENGAGE_BRAKE      = false;
    private const bool DISENGAGE_BRAKE   = true;

    ////////////////////////////////////////////////////////////////////////////
    // Target indices:
    ////////////////////////////////////////////////////////////////////////////

    public const int NUM_TARGETS = 1;
    private byte IDX_TARG_BASE   = 1;

    ////////////////////////////////////////////////////////////////////////////
    // Object instances:
    ////////////////////////////////////////////////////////////////////////////

    public static ReHandyBotController instance;
    private DistalComm distalRobot = new(); // Distal Control Library object

    ////////////////////////////////////////////////////////////////////////////
    // RHB control settings - CRITICAL
    // NOTE: use [RHB ctrl params - stability v5b game settings 4-axis.xlsx] to calculate damping as a function of stiffness
    ////////////////////////////////////////////////////////////////////////////

    private float FORCE_GAIN_RADIAL = 9f;
    private float FORCE_GAIN_ROT = 14f;

    private float K_STIFF_RADIAL_WALL = 2500f; // use with zero feedback gain
    private float B_DAMP_RADIAL_WALL  = 0f; // 40f; // rely on embedded HL_SetTarget stability

    private float K_STIFF_ROT_WALL    = 1.2f; // use with zero feedback gain
    private float B_DAMP_ROT_WALL     = 0f; // 0.092f; // rely on embedded HL_SetTarget stability

    private float POS_RADIAL_MIN      = 0.0145f;
    private float POS_RADIAL_MAX      = 0.06f;  

    private float POS_ROT_MIN = -Mathf.PI / 2f;
    private float POS_ROT_MAX =  Mathf.PI / 2f;

    // Throttle - BASELINE haptics settings:
    public const float POS_RADIAL_BASE = 0.029f; // used by MotorbikeController
    private float K_STIFF_RADIAL_BASE  = 2500f;
    private float B_DAMP_RADIAL_BASE   = 0f; // rely on embedded HL_SetTarget stability 

    // Steering - BASELINE haptics settings:
    private const float POS_ROT_BASE = 0f;
    private float K_STIFF_ROT_BASE   = 0.05f; // 0.1f; // 
    private float B_DAMP_ROT_BASE    = 0f; // rely on embedded HL_SetTarget stability

    // Stiffness for tracking control;
    private float K_ROT_STIFF_TRACKING = 1.0f; // rely on embedded HL_SetTarget stability

    ////////////////////////////////////////////////////////////////////////////
    // Impedance for RHB motion limits:
    ////////////////////////////////////////////////////////////////////////////   

    private float K_STIFF_ROT_LIM  = 0.6f;
    private float B_DAMP_ROT_LIM   = 0f; // rely on embedded HL_SetTarget stability

    public float ANGLE_ROT_LIM_DEG = 60.0f; // TODO: revise this value (too large?)

    ////////////////////////////////////////////////////////////////////////////
    // Data structures from bike and track objects:
    ////////////////////////////////////////////////////////////////////////////

    static Vector3 NULL_VECTOR3 = Vector3.zero;
    private const float NULL_VALUE      = 0f;

    // Track coordinates:
    private Vector3 pos_ctrline_near   = NULL_VECTOR3;
    private Vector3 vect_ctrline_tang  = NULL_VECTOR3;
    // private float curv_ctrline_near    = NULL_VALUE; // TODO: keep or discard
    // private float ang_ctrline_tang     = NULL_VALUE;
    // private float dist_ctrline_near    = NULL_VALUE; 

    // Bike pose:
    private float angle_roll_bike      = NULL_VALUE;
    // private float dt_angle_roll_bike   = NULL_VALUE;
    // private float angle_ctrl_signed    = NULL_VALUE;
    // private float dt_angle_ctrl_signed = NULL_VALUE;

    // Feedback control:
    // private float input_steer_targ = NULL_VALUE;
    private float angle_roll_targ  = NULL_VALUE;

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

    private bool isSystemStarted    = false;
    public bool  isExerciseStarted  = false; // changed to public for access by DataManager (27.08.2025)
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
    private bool allowCalibrate = false;

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

    public float timeElapsedValue = 0f;

    public int step_count = 0;

    ////////////////////////////////////////////////////////////////////////////
    // Thread and timer for SetTarget process:
    ////////////////////////////////////////////////////////////////////////////

    private System.Timers.Timer timerSetTarget;
    private Thread threadTimerSetTarget;

    ////////////////////////////////////////////////////////////////////////////
    // Data display:
    ////////////////////////////////////////////////////////////////////////////

    // private int  DT_DISP_DATA_MSEC      = 1000;
    // private bool DISP_TIMER_ACTIVITY_ON = true;

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // METHODS SECTION:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    // Application start / stop functions:
    ////////////////////////////////////////////////////////////////////////////

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

    private void Destroy()
    {
        // StopSetTargetEvents() - removed 30.08.2025
    }

    private void ConnectRHB()
    {
        connectionTween?.Kill();
        connectionTween = DOVirtual.DelayedCall(10f, ReConnectRHB);

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
                    ReConnectRHB();
                }
            });
        });
        connectionThread.Start();
    }

    private void ReConnectRHB()
    {
        if (RHBConnected)
        {
            StartSystem(OnConnect);
            return;
        }
        ConnectRHB();
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

    ////////////////////////////////////////////////////////////////////////////
    // Pre-game 'on complete" actions:
    ////////////////////////////////////////////////////////////////////////////
    
    private void OnConnect()
    {
        SetBrakes(DISENGAGE_BRAKE, DISENGAGE_BRAKE);
        ExternalConsoleLogger.Log("        OnConnect(): SetBrakes(): cmd DISENGAGE \n");

        loader.SetActive(true);

        if (STATE_PREGAME == ST_SET_CTRL_MODE)
        {
            loaderText.text =
               "CLICK on this screen and \n\n" +
               "Select CONTROL MODE: \n\n" +
               "(1) ASSISTED CONTROL \n" +
               "(2) AUTO STEER / AUTO THROTTLE \n" +
               "(3) AUTO STEER / MANUAL THROTTLE \n" +
               "(4) PURE MANUAL";
        }

        loaderText.alignment = TextAlignmentOptions.MidlineLeft;
    }

    private void OnSelectGameSettings()
    {

        if (STATE_PREGAME == ST_SET_FACT_ASSIST_STEER) 
        {
            loaderText.text =
                "Select STEER ASSISTANCE level (0 to 9) \n";
        }

        else if (STATE_PREGAME == ST_SET_FACT_ASSIST_THROTTLE)
        {
            loaderText.text =
                "Select THROTTLE mode: \n" +
                "(0) MANUAL throttle \n" +
                "(1) AUTO throttle \n";
        }

        else if (STATE_PREGAME == ST_CALIBRATE)
        {
            string str_fact_assist;

            if (CASE_CTRL_MODE == CTRL_ASSISTED)
                str_fact_assist = 
                    "Assist factor STEERING = " + FACT_ASSIST_STEER + "\n" +                    
                    "Assist factor THROTTLE = " + FACT_ASSIST_THROTTLE;
            else
                str_fact_assist = " ";

            loaderText.text =
            "CONTROL MODE [" + CASE_CTRL_MODE + "]\n" +
             str_fact_assist + "\n\n" +
            "Align grippers horizontally and close the grippers \n\n" +
            "Press Y to CALIBRATE";
        }
    }

    private void OnCalibrate_CmdStartExercise()
    {
        allowCalibrate = false;

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

    ////////////////////////////////////////////////////////////////////////////
    // Pre-game actions (Obtain game modes / Calibrate):
    ////////////////////////////////////////////////////////////////////////////

    private void SelectGameSettings(UnityAction onComplete = null)
    {
        ////////////////////////////////////////////////
        // Select CONTROL MODE:
        ////////////////////////////////////////////////

        if (STATE_PREGAME == ST_SET_CTRL_MODE)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1))
                CASE_CTRL_MODE = CTRL_ASSISTED;
            else if (Input.GetKeyDown(KeyCode.Alpha2))
                CASE_CTRL_MODE = CTRL_AUTO_STEER_AUTO_THROT;
            else if (Input.GetKeyDown(KeyCode.Alpha3))
                CASE_CTRL_MODE = CTRL_AUTO_STEER_MANUAL_THROT;
            else if (Input.GetKeyDown(KeyCode.Alpha4))
                CASE_CTRL_MODE = CTRL_MANUAL_SIMPLE;
            else
                return;

            if (CASE_CTRL_MODE == CTRL_ASSISTED)
                STATE_PREGAME = ST_SET_FACT_ASSIST_STEER;
            else
                STATE_PREGAME = ST_CALIBRATE;

            onComplete.Invoke();
        }

        ////////////////////////////////////////////////
        // Select STEERING assistance factor:
        ////////////////////////////////////////////////

        else if (STATE_PREGAME == ST_SET_FACT_ASSIST_STEER)
        {
            if (Input.GetKeyDown(KeyCode.Alpha0))
                FACT_ASSIST_STEER = 0f;
            else if (Input.GetKeyDown(KeyCode.Alpha1))
                FACT_ASSIST_STEER = 1f;
            else if (Input.GetKeyDown(KeyCode.Alpha2))
                FACT_ASSIST_STEER = 2f;
            else if (Input.GetKeyDown(KeyCode.Alpha3))
                FACT_ASSIST_STEER = 3f;
            else if (Input.GetKeyDown(KeyCode.Alpha4))
                FACT_ASSIST_STEER = 4f;
            else if (Input.GetKeyDown(KeyCode.Alpha5))
                FACT_ASSIST_STEER = 5f;
            else if (Input.GetKeyDown(KeyCode.Alpha6))
                FACT_ASSIST_STEER = 6f;
            else if (Input.GetKeyDown(KeyCode.Alpha7))
                FACT_ASSIST_STEER = 7f;
            else if (Input.GetKeyDown(KeyCode.Alpha8))
                FACT_ASSIST_STEER = 8f;
            else if (Input.GetKeyDown(KeyCode.Alpha9))
                FACT_ASSIST_STEER = 9f;
            else
                return;

            // Convert assistance factor to fraction:
            FACT_ASSIST_STEER /= 10f;

            STATE_PREGAME = ST_SET_FACT_ASSIST_THROTTLE;
            onComplete.Invoke();
        }

        ////////////////////////////////////////////////
        // Select STEERING assistance factor:
        ////////////////////////////////////////////////

        else if (STATE_PREGAME == ST_SET_FACT_ASSIST_THROTTLE)
        {
            if (Input.GetKeyDown(KeyCode.Alpha0))
                FACT_ASSIST_THROTTLE = 0f;
            else if (Input.GetKeyDown(KeyCode.Alpha1))
                FACT_ASSIST_THROTTLE = 1f;
            else
                return;

            STATE_PREGAME = ST_CALIBRATE;
            onComplete.Invoke();
        }
    }

    private void Calibrate(UnityAction onComplete = null)
    {
        for (int i = 0; i < MaxAttempts; i++)
            if (distalRobot.Calibration(DistalComm.CalibrationType.AxisCalib)) break;

        /*
        for (int i = 0; i < MaxAttempts; i++)
            if (distalRobot.Calibration(DistalComm.CalibrationType.AllForceSensorsZeroCalib)) break;
        */

        onComplete.Invoke();
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // Real-time update - CRITICAL:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    private void Update()
    {   
        ////////////////////////////////////////////////////////////////////////////
        // Allow the user to select game modes:
        ////////////////////////////////////////////////////////////////////////////

        if (STATE_PREGAME != ST_CALIBRATE)
            SelectGameSettings(OnSelectGameSettings);            

        else if (Input.GetKeyDown(KeyCode.Y))
            Calibrate(OnCalibrate_CmdStartExercise);  

        ////////////////////////////////////////////////////////////////////////////
        // Toggle Exercise state:
        ////////////////////////////////////////////////////////////////////////////

        // If robot is calibrated and user presses Enter, Exercise state is be toggled
        // Exercise will start now if not already running

        if (isCalibrated && Input.GetKeyDown(KeyCode.Return))
            ToggleExerciseState();

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
        float pos_rot    = ReHandyBotController.instance.distal_data.PositionP;

        ////////////////////////////////////////////////////////////////////////////
        // Retrieve data from bike and track objects:
        ////////////////////////////////////////////////////////////////////////////

        if (ExerciseActive && MotorbikeController.instance != null && Track.instance != null)
        {
            // Retrieve bike pose coordinates:
            angle_roll_bike = MotorbikeController.instance.bike_pose_data.angle_roll_bike;

            // Retrieve fedback control data:
            angle_roll_targ = MotorbikeController.instance.fbk_ctrl_data.angle_roll_targ;
        }

        ////////////////////////////////////////////////////////////////////////////
        // Set Target commands for steering and throttle - CRITICAL:
        //////////////////////////////////////////////////////////////////////////// 
        
        CmdSetTargetSteerAndThrottleCases(pos_rot, angle_roll_targ, angle_roll_bike, CASE_CTRL_MODE);

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
    // Force feedback gains - CASES:
    ////////////////////////////////////////////////////////////////////////////

    void SetForceFeebackGainCases(int case_ctrl_mode) { 
        switch (case_ctrl_mode)
        {
            case CTRL_ASSISTED:

                SetGain(
                    FORCE_GAIN_RADIAL, 
                    FORCE_GAIN_ROT);
                break;

            case CTRL_AUTO_STEER_AUTO_THROT:
            case CTRL_AUTO_STEER_MANUAL_THROT:

                SetGain(0f, 0f);
                break;

            case CTRL_MANUAL_SIMPLE:

                SetGain(
                    FORCE_GAIN_RADIAL, 
                    FORCE_GAIN_ROT);
                break;

            default: // no command
                break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Control commands using Set Target - CASES:
    ////////////////////////////////////////////////////////////////////////////

    void CmdSetTargetSteerAndThrottleCases(float pos_rot, float angle_roll_targ_this, float angle_roll_bike_this, int case_ctrl_mode) {

        switch (case_ctrl_mode)
        {
            case CTRL_ASSISTED:

                const float FRAC_ROT_EXCESS = 0.5f; 

                float pos_rot_eq_ref    = FRAC_POS_ROT_INPUT_USER * angle_roll_targ_this;
                float k_stiff_rot_steer = FACT_ASSIST_STEER * K_ROT_STIFF_TRACKING;

                // Reduce tracking stiffness if user exceeds reference position:
                if (Math.Abs(pos_rot) > Math.Abs(pos_rot_eq_ref))
                    k_stiff_rot_steer *= FRAC_ROT_EXCESS;

                CmdSetTarget_FeedbackCtrl_WithLimit(
                    pos_rot,
                    pos_rot_eq_ref, 
                    k_stiff_rot_steer);
                break;

            case CTRL_AUTO_STEER_AUTO_THROT:
            case CTRL_AUTO_STEER_MANUAL_THROT:

                CmdSetTarget_AutoSteer(
                    FRAC_POS_ROT_INPUT_USER * angle_roll_bike_this,
                    K_ROT_STIFF_TRACKING);
                break;

            case CTRL_MANUAL_SIMPLE:

                CmdSetTarget_FeedbackCtrl_WithLimit(
                    pos_rot,
                    0f, 
                    K_STIFF_ROT_BASE);
                break;

            default: // no command
                break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Control commands using Set Target - functions:
    ////////////////////////////////////////////////////////////////////////////

    // TODO: keep or discard:
    /*
    private void CmdSetTargetCtrl_ManualSimple_WithLimit(float pos_rot)
    {
        ////////////////////////////////////////////////////////////////////////////
        // Impedance parameters for rotation angle limit:
        ////////////////////////////////////////////////////////////////////////////

        // RADIAL parameters:
        float switch_radial = 1.0f;

        // ROTATIONAL parameters:
        float ANGLE_ROT_LIM = FACT_DEG_2_RAD * ANGLE_ROT_LIM_DEG;

        float pos_rot_eq_curr;
        float k_stiff_rot_curr;
        float b_damp_rot_curr;

        float switch_rot = 1.0f;

        ////////////////////////////////////////////////////////////////////////////
        // Compute impedance parameters:
        // If rotation limit is exceeded, apply limit values to rotational stiffness and damping:
        ////////////////////////////////////////////////////////////////////////////

        // NEW IMPLEMENTATION:
        // Compute equivalent stiffnes & equilibrium point for the combined steering stiffness and limit-position stiffness
        // Overcomes the limitation of RHB only allowing a single target (22.08.2025):

        float pos_rot_eq_lim_plus  =  ANGLE_ROT_LIM;
        float pos_rot_eq_lim_minus = -ANGLE_ROT_LIM;

        if (pos_rot > pos_rot_eq_lim_plus)
        {
            k_stiff_rot_curr = K_STIFF_ROT_BASE_STEER + K_STIFF_ROT_LIM;
            pos_rot_eq_curr = (K_STIFF_ROT_LIM * pos_rot_eq_lim_plus) / k_stiff_rot_curr;
        }

        else if (pos_rot < pos_rot_eq_lim_minus)
        {
            k_stiff_rot_curr = K_STIFF_ROT_BASE_STEER + K_STIFF_ROT_LIM;
            pos_rot_eq_curr = (K_STIFF_ROT_LIM * pos_rot_eq_lim_minus) / k_stiff_rot_curr;
        }
        else
        {
            k_stiff_rot_curr = K_STIFF_ROT_BASE_STEER;
            pos_rot_eq_curr = 0f;
        }

        // Assumes that damping is provided by embedded HL_SetTarget stability (22.08.2025):
        b_damp_rot_curr = 0f;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        if (ExerciseActive)
            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_BASE_THROT, pos_rot_eq_curr,
                K_STIFF_RADIAL_BASE_THROT, k_stiff_rot_curr,
                B_DAMP_RADIAL_BASE_THROT, b_damp_rot_curr,
                switch_radial, switch_rot);
        else
            success_set_target = false;
    }
    */

    private void CmdSetTarget_AutoSteer(float pos_rot_eq_ref, float k_stiff_rot_steer)
    {
        ////////////////////////////////////////////////////////////////////////////
        // Impedance parameters for rotation angle limit:
        ////////////////////////////////////////////////////////////////////////////

        // RADIAL parameters:
        float SWITCH_RADIAL = 1f;

        // ROTATIONAL parameters:
        float SWITCH_ROT = 1;
        float b_damp_rot_steer = 0f; // Assumes that damping is provided by embedded HL_SetTarget stability

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        if (ExerciseActive)
            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_BASE, pos_rot_eq_ref,
                K_STIFF_RADIAL_BASE, k_stiff_rot_steer,
                B_DAMP_RADIAL_BASE, b_damp_rot_steer,
                SWITCH_RADIAL, SWITCH_ROT);
        else
            success_set_target = false;        
    }

    private void CmdSetTarget_FeedbackCtrl_WithLimit(float pos_rot, float pos_rot_eq_ref, float k_stiff_rot_steer)
    {
        ////////////////////////////////////////////////////////////////////////////
        // RADIAL parameters:
        ////////////////////////////////////////////////////////////////////////////
        
        const float SWITCH_RADIAL = 1.0f;

        ////////////////////////////////////////////////////////////////////////////
        // ROTATIONAL parameters:
        ////////////////////////////////////////////////////////////////////////////        

        const float SWITCH_ROT = 1.0f;

        float ANGLE_ROT_LIM = FACT_DEG_2_RAD * ANGLE_ROT_LIM_DEG;

        // Equivalent impedance - combining reference (trajectory) equilibrium position and rotation limit position:
        float pos_eq_rot_equiv;
        float k_stiff_rot_equiv;
        float b_damp_rot_equiv;

        ////////////////////////////////////////////////////////////////////////////
        // Compute equivalent impedance parameters:
        ////////////////////////////////////////////////////////////////////////////

        // TODO: remove at a later date:
        /*
        float pos_rot_eq_lim_plus  =  ANGLE_ROT_LIM;
        float pos_rot_eq_lim_minus = -ANGLE_ROT_LIM;

        if (pos_rot > pos_rot_eq_lim_plus)
        {
            k_stiff_rot_equiv = k_stiff_rot_steer + K_STIFF_ROT_LIM;

            pos_eq_rot_equiv  = (k_stiff_rot_steer * pos_rot_eq_ref + K_STIFF_ROT_LIM * pos_rot_eq_lim_plus) 
                / k_stiff_rot_equiv;
        }

        else if (pos_rot < pos_rot_eq_lim_minus)
        {
            k_stiff_rot_equiv = k_stiff_rot_steer + K_STIFF_ROT_LIM;

            pos_eq_rot_equiv  = (k_stiff_rot_steer * pos_rot_eq_ref + K_STIFF_ROT_LIM * pos_rot_eq_lim_minus) 
                / k_stiff_rot_equiv;
        }
        else
        {
            k_stiff_rot_equiv = k_stiff_rot_steer;
            pos_eq_rot_equiv  = pos_rot_eq_ref;
        }
        */

        float pos_rot_eq_lim;
        float k_stiff_rot_lim;

        if (pos_rot > ANGLE_ROT_LIM) 
        {
            pos_rot_eq_lim  = ANGLE_ROT_LIM;
            k_stiff_rot_lim = K_STIFF_ROT_LIM;
        }

        else if (pos_rot < -ANGLE_ROT_LIM) 
        { 
            pos_rot_eq_lim  = -ANGLE_ROT_LIM;
            k_stiff_rot_lim = K_STIFF_ROT_LIM;
        }

        else 
        {
            pos_rot_eq_lim  = 0f; // dummy value
            k_stiff_rot_lim = 0f; // true value
        }

        k_stiff_rot_equiv = k_stiff_rot_steer + k_stiff_rot_lim + K_STIFF_ROT_BASE;

        pos_eq_rot_equiv = (k_stiff_rot_steer*pos_rot_eq_ref + k_stiff_rot_lim*pos_rot_eq_lim)
             / k_stiff_rot_equiv;

        // Assumes that damping is provided by embedded HL_SetTarget stability (22.08.2025):
        b_damp_rot_equiv = 0f;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        if (ExerciseActive)
            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_BASE, pos_eq_rot_equiv,
                K_STIFF_RADIAL_BASE, k_stiff_rot_equiv,
                B_DAMP_RADIAL_BASE, b_damp_rot_equiv,
                SWITCH_RADIAL, SWITCH_ROT);
        else
            success_set_target = false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Ancillary functions - RHB control:
    ////////////////////////////////////////////////////////////////////////////

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
            SetForceFeebackGainCases(CASE_CTRL_MODE);

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
            { ...
        */
    }

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
                POS_RADIAL_BASE, POS_ROT_BASE,
                K_STIFF_RADIAL_BASE, K_STIFF_ROT_BASE,
                B_DAMP_RADIAL_BASE, B_DAMP_ROT_BASE,
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

    // This is for usage for SetOffsetForces command, currently being called with dummy values
    private void SetOffsetForces()
    {
        ReHandyBotController.instance.SetOffsetForces(0f, 0f);
    }



    // SetTarget functions removed 30.08.2025:
    /*
    private void SetupSetTargetEvents()
   
    private void StartSetTargetEvents()

    private void StopSetTargetEvents()

    private void SendCmdSetTargetSteerLimit(object sender, ElapsedEventArgs e)
    */

}