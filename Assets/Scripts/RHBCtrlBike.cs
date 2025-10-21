using Articares.Distal;
using DG.Tweening;
using UnityEngine.UI;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
//using System.Runtime.Remoting.Messaging;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.SceneManagement;
using PimDeWitte.UnityMainThreadDispatcher;
// using UnityEngine.Video;

public class RHBCtrlBike : MonoBehaviour
{
    float FACT_DEG_2_RAD = (float)Math.PI / 180f;

    ////////////////////////////////////////////////////////////////////////////
    // Application time step - CRITICAL:
    ////////////////////////////////////////////////////////////////////////////

    public const int DT_STEP_APP_MSEC = 25;

    ////////////////////////////////////////////////////////////////////////////
    // Real-time control flags - for debugging:
    ////////////////////////////////////////////////////////////////////////////

    public const bool USE_RT_TIMER_LOCK = false;
    public const bool USE_STANDALONE_UI = true; // make false for Care Platform game

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - Game control modes:
    //////////////////////////////////////////////////////////////////////////// 

    const int NULL_SETTING = -1;

    // Bike type (BEGINNER or PRO; default is PRO)
    // To be set by UNITY_GAME or CARE_PLATFORM (no need to implement selection for first CARE_PLATFORM release)
    public bool USE_BEGINNER_BIKE_CONSTR = false;

    public const int CTRL_ASSISTED = 1;
    public const int CTRL_AUTO_STEER_AUTO_THROT = 2;
    public const int CTRL_AUTO_STEER_MANUAL_THROT = 3;
    public const int CTRL_MANUAL_SIMPLE = 4;

    // Bike control mode (default is ASSISTED)
    // To be set by UNITY_GAME or CARE_PLATFORM (no need to implement selection now, keep it as CTRL_ASSISTED for first CARE_PLATFORM release)
    public int CASE_CTRL_MODE = CTRL_ASSISTED;

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - MANUAL throttle parameters:
    //////////////////////////////////////////////////////////////////////////// 

    // Handles opening distance (meters) for zero throttle input:
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient hand opening data; keep default for first CARE_PLATFORM release)
    public float POS_RADIAL_THROT_ZERO = 0.029f;

    // Throttle stiffness for MANUAL throttle mode:
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient stiffness calibration data; keep default for first CARE_PLATFORM release)
    public float K_STIFF_RADIAL_THROT_MANUAL;

    // Offset of the THROTTLE zero position to account for initial calibration errors (PD discussion on 20.09.2025)
    public float OFFS_POS_RADIAL_CALIB = 0.0005f;
    public float POS_RADIAL_THROT_ZERO_OFFS;

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - AUTO throttle parameters:
    //////////////////////////////////////////////////////////////////////////// 

    // AUTO throttle speed limit in kph:
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM game level)
    public float SPEED_AUTO_THROTTLE_MAX_KPH; // 150f; //

    ////////////////////////////////////////////////////////////////////////////
    // UNITY_GAME: states for PRE-GAME procedures:
    //////////////////////////////////////////////////////////////////////////// 

    public const int ST_SELECT_BIKE_TYPE = 1;
    public const int ST_SET_CTRL_MODE = 2;
    public const int ST_SET_FACT_ASSIST_STEER = 3;
    public const int ST_SET_FACT_ASSIST_THROTTLE = 4;
    public const int ST_CALIBRATE = 5;
    public const int ST_RHB_READY = 6;
    // public const int ST_EXERCISE_ACTIVE = 7;

    public int STATE_PREGAME = ST_SELECT_BIKE_TYPE; // initial state for UNITY_GAME procedures

    ////////////////////////////////////////////////////////////////////////////
    // SetExercise() parameters:
    //////////////////////////////////////////////////////////////////////////// 

    private float OFFS_FORCE_RADIAL_INIT = 0f;
    private float OFFS_TORQUE_ROT_INIT = 0f;

    private bool SAFETY_TCP_APP_ON = false;
    private bool STABILITY_SET_TARG_ON = true;

    public const bool ENGAGE_BRAKE = false;
    public const bool DISENGAGE_BRAKE = true;

    ////////////////////////////////////////////////////////////////////////////
    // Target indices:
    ////////////////////////////////////////////////////////////////////////////

    public const int NUM_TARGETS = 1;
    private const byte IDX_TARG_BASE = 1;

    ////////////////////////////////////////////////////////////////////////////
    // Object instances:
    ////////////////////////////////////////////////////////////////////////////

    public static RHBCtrlBike instance;
    private DistalComm distalRobot = new(); // Distal Control Library object

    ////////////////////////////////////////////////////////////////////////////
    // RHB control settings - CRITICAL
    // NOTE: use [RHB ctrl params - stability v5b game settings 4-axis.xlsx] to calculate damping as a function of stiffness
    ////////////////////////////////////////////////////////////////////////////

    public float FORCE_GAIN_RADIAL = 9.0f;
    public float FORCE_GAIN_ROT = 6.0f; // 14.0f; // Reduced gain for greater stability with ASSISTED and MANUAL control

    private float K_STIFF_RADIAL_WALL = 2500f; // use with zero feedback gain

    // Extra damping to prevent limit cycles when handles contact physical limit
    // Normally should rely on embedded HL_SetTarget stability
    // TODO: check why limit cycle suppression doesn't act in firmware (29.09.2025):
    private float B_DAMP_RADIAL_WALL = 10f;

    private float K_STIFF_ROT_WALL = 1.2f; // use with zero feedback gain
    private float B_DAMP_ROT_WALL = 0f; // rely on embedded HL_SetTarget stability

    public float POS_RADIAL_MIN = 0.0145f;
    public float POS_RADIAL_MAX = 0.06f;

    // Offset of the MINIMUM RADIAL position to account for initial calibration errors (PD discussion on 20.09.2025):
    public float POS_RADIAL_MIN_OFFS;

    private float POS_ROT_MIN = -Mathf.PI / 2f;
    private float POS_ROT_MAX = Mathf.PI / 2f;

    // Throttle - additional haptics settings:
    public float K_STIFF_RADIAL_THROT_AUTO = 5000f; // makes handles essentially rigid
    public float B_DAMP_RADIAL_BASE = 0f; // rely on embedded HL_SetTarget stability 

    // Steering - BASELINE haptics settings:
    public const float POS_ROT_BASE = 0f;

    public float K_STIFF_ROT_BASE = 0.05f; // 0.1f; // 
    public float B_DAMP_ROT_BASE = 0f; // rely on embedded HL_SetTarget stability

    // Stiffness for TRACKING control
    // NOTE: check stabilizing damping for K_STIFF_ROT_TRACKING + K_STIFF_ROT_BASE
    // (use RHB ctrl params - stability v5b game settings 4-axis)
    public float K_STIFF_ROT_TRACKING = 2.2f;
    public float B_DAMP_ROT_TRACKING = 0.05f; // 0.045f; // 0.040f; // 

    // Stiffness for ASSISTIVE control - fraction of TRACKING stiffness:
    public float FRAC_ASSIST_STIFF = 0.5f; // 0.45f; // 0.35f; // 

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - STEERING assistance:
    //////////////////////////////////////////////////////////////////////////// 

    // Maximum STEERING ASSIST torque - CRITICAL:
    public const float TORQUE_ASSIST_STEER_MAX = 0.18f; // 0.2f; // 0.1f;

    // ASSIST FACTOR (between 0 and 1.0) - modified computation after user feedbacks (29.09.2025)
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM game level)
    public float FACT_ASSIST_STEER = 0f;

    public const int GAME_LEVEL_MID = 5;
    public const float FACT_ASSIST_MID = 0.3f;

    // Scaling factor for Patient's rotational inputs
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient ROM data)
    public float FRAC_POS_ROT_INPUT_PATIENT = 0.4f;

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - THROTTLE assistance:
    //////////////////////////////////////////////////////////////////////////// 

    public const int THROTTLE_MODE_MANUAL = 0;
    public const int THROTTLE_MODE_AUTO = 1;

    // Throttle mode (0: MANUAL, 1: AUTO, default is AUTO)
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM throttle setting)
    public float FACT_ASSIST_THROTTLE = (float)THROTTLE_MODE_AUTO;

    ////////////////////////////////////////////////////////////////////////////
    // RHB motion limits: stiffness & angle
    ////////////////////////////////////////////////////////////////////////////   

    private float K_STIFF_ROT_LIM = 0.6f;

    public float ANGLE_ROT_LIM_DEG = 45f;

    ////////////////////////////////////////////////////////////////////////////
    // Kinematic & force data variables:
    ////////////////////////////////////////////////////////////////////////////

    static Vector3 NULL_VECTOR3 = Vector3.zero;
    private const float NULL_VALUE = 0f;

    // Track coordinates:
    private Vector3 pos_ctrline_near = NULL_VECTOR3;
    private Vector3 vect_ctrline_tang = NULL_VECTOR3;

    // Bike pose:
    private float angle_roll_bike = NULL_VALUE;
    private float dt_angle_roll_bike = NULL_VALUE;

    // Feedback control:
    private float angle_roll_targ = NULL_VALUE;
    private float dt_angle_roll_targ = NULL_VALUE;

    // Trajectory tracking: reference equilibrium position:
    public float pos_rot_eq_ref = NULL_VALUE;
    public float dt_pos_rot_eq_ref = NULL_VALUE;

    // Assistive torque:
    public float torque_assist = NULL_VALUE;

    ////////////////////////////////////////////////////////////////////////////
    // RHB info related variables:
    ////////////////////////////////////////////////////////////////////////////
    private bool RHBConnected => distalRobot.is_device_connected;
    public DistalComm.ExerciseData distal_data => distalRobot.DistalData;

    ////////////////////////////////////////////////////////////////////////////
    // Exercise-related variables:
    ////////////////////////////////////////////////////////////////////////////

    private bool isSystemStarted = false;

    public bool isExerciseStarted = false; // changed to public for access by MotorbikeController (27.08.2025)

    // Exercise START / STOP procedures - acivation flags (15.10.2025):
    private bool runProceduresExerciseStart = false;
    private bool runProceduresExerciseStop = false;

    // Flag to maintain 'upright' constraint while exercise is inactive and throttle input is zero:
    public bool UPRIGHT_CONSTR_ON; // constraint flag (13.09.2025)

    ////////////////////////////////////////////////////////////////////////////
    // Constants:
    ////////////////////////////////////////////////////////////////////////////

    private const int MAX_ATTEMPTS = 10;

    private const string ServerIP = "192.168.102.1";
    private const int ServerPort = 3002;

    ////////////////////////////////////////////////////////////////////////////
    // CONNECTION thread:
    ////////////////////////////////////////////////////////////////////////////

    private Thread connectionThread;
    private Tween connectionTween;

    ////////////////////////////////////////////////////////////////////////////
    // REAL-TIME RHB control thread:
    //////////////////////////////////////////////////////////////////////////// 

    // Thread states:
    const ThreadState THREAD_RUNNING = (ThreadState)0;

    private Queue<Action> mainThreadActionQueue = new();

    private Thread rtControlThread = null;
    private bool enabledControlThread = false;

    ////////////////////////////////////////////////////////////////////////////
    // REAL-TIME RHB control loop timers:
    ////////////////////////////////////////////////////////////////////////////

    private bool timerActive = false;
    private bool timerActivePrev = false;
    private bool timerLocked = false;

    public float timeElapsedValue = 0f;

    public int step_count;

    ////////////////////////////////////////////////////////////////////////////
    // Game control variables:
    ////////////////////////////////////////////////////////////////////////////

    public bool isCalibrated = false;

    public Action OnExerciseStart;
    public Action OnExerciseStop;

    private const string PrototypeSceneName = "Prototype";

    //////////////////////////////////////////////////////////////////////////// 
    // NEW: GAME LEVELS (29.09.2025)
    //////////////////////////////////////////////////////////////////////////// 

    public const int N_GAME_LEVELS = 10;

    public int game_level_curr = 1; // default value
    public int game_level_next;

    ////////////////////////////////////////////////////////////////////////////
    // NEW: RACE DIRECTION (29.09.2025)
    //////////////////////////////////////////////////////////////////////////// 

    public const int DIR_CW = +1; // clockwise direction
    public const int DIR_CCW = -1; // counterclockwise direction 

    public int RACE_DIRECTION;

    ////////////////////////////////////////////////////////////////////////////
    // NEW: PERFORMANCE variables (29.09.2025)
    ////////////////////////////////////////////////////////////////////////////

    // UNDERSTEER parameters:
    public const float FRAC_UNDERSTEER_LEVEL_UP_MAX = 0.33f; // if understeer fraction is less than this, go up one level
    public const float FRAC_UNDERSTEER_LEVEL_DOWN_MIN = 0.67f; // if understeer fraction is more than this, go down one level

    public float frac_understeer = -1f;

    // DISTANCE TRAVELED parameters:
    // "Legitimate" race for performace purposes
    // Traveled distance must be greater than this fraction of the track length 
    public const float FRAC_LENGTH_TRACK_LEGIT_RACE = 0.9f;

    public float frac_dist_traveled = 0f;

    // FALL parameters:
    // Number of falls allowed without triggering GAME LEVEL reduction:   
    public const int N_FALLS_LIM = 1;

    // Game level change value (-1, 0, +1): 
    public int game_level_change = 0;

    ////////////////////////////////////////////////////////////////////////////
    // Brake variables:
    ////////////////////////////////////////////////////////////////////////////

    const int N_ATTEMPTS_BRAKE = 50;
    const int DELAY_BRAKE_MSEC = 20;

    ////////////////////////////////////////////////////////////////////////////
    // Loader variables - TODO: why don't they work if placed in BikeGameUI?
    ////////////////////////////////////////////////////////////////////////////

    [Space]
    [Header("UI")]
    [SerializeField] public GameObject loader;
    // [SerializeField] public GameObject exerciseGuidelineText;
    [SerializeField] public TMP_Text loaderText;

    int connect_count = 0; // connection attempts counter - for debugging

    ////////////////////////////////////////////////////////////////////////////
    // Display variables:
    ////////////////////////////////////////////////////////////////////////////

    const bool DISP_RT_LOOP_ON = true;
    const bool DISP_CONSOLE_ON = true;

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // METHODS SECTION:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    // FUNCTIONS TO HARMONIZE with ReHandyBotController (10.10.2025):
    ////////////////////////////////////////////////////////////////////////////

    private void Awake()
    {
        // Singleton logic:
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
        //System.Diagnostics.Process.Start("ethernet_reset.bat");

        ////////////////////////////////////////////////////////////////////////////
        // Launch connection thread - CRITICAL:
        ////////////////////////////////////////////////////////////////////////////

        if (USE_STANDALONE_UI)
            while (BikeGameUI.instance == null)
                Task.Delay(10);

        // ConnectRHB();
        ConnectRHBSimple();

        while (mainThreadActionQueue.Count > 0)
            mainThreadActionQueue.Dequeue().Invoke();
    }

    private void OnApplicationQuit()
    {
        // Destroy connnection thread:
        connectionThread?.Abort();
        connectionTween?.Kill();

        // Destroy real-time control thread:
        enabledControlThread = false;
        rtControlThread?.Join();

        if (distalRobot == null || !RHBConnected)
            return;

        if (isExerciseStarted) distalRobot.StopExercise();
        if (isSystemStarted) distalRobot.StopSystem();
        if (RHBConnected) distalRobot.CloseConnection();

        distalRobot = null;
        instance = null;
    }
    private bool EstablishConnection(UnityAction onComplete = null)
    {
        if (RHBConnected)
        {
            onComplete?.Invoke();
            return true;
        }

        for (int i = 0; i < MAX_ATTEMPTS; i++)
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

        for (int i = 0; i < MAX_ATTEMPTS; i++)
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
    // Game-specific RHB functions:
    ////////////////////////////////////////////////////////////////////////////

    public void ConnectRHBSimple()
    {
        const float CALL_DELAY_VALUE = 1.0f;

        if (DISP_CONSOLE_ON)
        {
            ExternalConsoleLogger.Log("__________________________________________");
            ExternalConsoleLogger.Log("ConnectRHBSimple() call count [" + connect_count++ + "]");
        }

        ////////////////////////////////////////////////////////////////////////////
        // Start CONNECTION thread:
        ////////////////////////////////////////////////////////////////////////////

        connectionThread?.Abort();
        connectionThread = new Thread(() =>
        {
            //////////////////////////////////////////////////////////////////
            // Activate loader:
            //////////////////////////////////////////////////////////////////

            if (USE_STANDALONE_UI)
                BikeGameUI.instance.SetLoaderState(true);

            // Display section:
            if (DISP_CONSOLE_ON)
                ExternalConsoleLogger.Log("ConnectRHBSimple() set loader");

            //////////////////////////////////////////////////////////////////
            // Attempt to establish RHB connection:
            //////////////////////////////////////////////////////////////////

            EstablishConnection(); // NOTE: this sets RHBConnected - no need for return value

            // Display section:
            if (DISP_CONSOLE_ON)
            {
                if (RHBConnected)
                    ExternalConsoleLogger.Log("ConnectRHBSimple() result: CONNECTED \n");
                else
                    ExternalConsoleLogger.Log("ConnectRHBSimple() result: MAKE NEW ATTEMPT \n");
            }

            //////////////////////////////////////////////////////////////////
            // Next action: start system or make new connection attempt
            //////////////////////////////////////////////////////////////////

            if (RHBConnected)
            {
                //////////////////////////////////////////////////////////////////
                // Deactivate loader:
                //////////////////////////////////////////////////////////////////

                if (USE_STANDALONE_UI)
                    BikeGameUI.instance.SetLoaderState(false);

                //////////////////////////////////////////////////////////////////
                // Start system:
                //////////////////////////////////////////////////////////////////

                StartSystem(() =>
                {

                    // Disengage brakes:
                    bool success = SetBrakesRHB(DISENGAGE_BRAKE, DISENGAGE_BRAKE, N_ATTEMPTS_BRAKE, DELAY_BRAKE_MSEC);

                    // Display section:
                    if (DISP_CONSOLE_ON)
                        ExternalConsoleLogger.Log("ConnectRHBSimple(): DISENGAGE BRAKES success = [" + success + "]\n");
                });

                BikeGameUI.instance.OnConnect_PreUnityGame();
            }
            else
            {
                connectionTween?.Kill();
                UnityMainThreadDispatcher.Instance().Enqueue(() => connectionTween = DOVirtual.DelayedCall(CALL_DELAY_VALUE, ConnectRHBSimple));
            }
        });

        connectionThread.Start();
    }

    /*
    public void ConnectRHB()
    {
        const float CALL_DELAY_VALUE = 10f;

        ////////////////////////////////////////////////////////////////////////////
        // Start CONNECTION thread:
        ////////////////////////////////////////////////////////////////////////////
        
        connectionTween?.Kill();
        connectionTween = DOVirtual.DelayedCall(CALL_DELAY_VALUE, ReConnectRHB);

        connectionThread?.Abort();
        connectionThread = new Thread(() =>
        {
            mainThreadActionQueue.Enqueue(() =>
            {
                if (USE_STANDALONE_UI)
                    BikeGameUI.instance.SetLoaderState(true);
            });

            // Establish RHB connection:
            bool success = EstablishConnection();

            mainThreadActionQueue.Enqueue(() =>
            {
                connectionTween?.Kill();

                if (success)
                {
                    if (USE_STANDALONE_UI)
                        BikeGameUI.instance.SetLoaderState(false);

                    StartSystem(BikeGameUI.instance.OnConnect_PreUnityGame);
                }
                else
                    ReConnectRHB();
            });
        });

        connectionThread.Start();
    }

    private void ReConnectRHB()
    {
        if (RHBConnected)
            StartSystem(BikeGameUI.instance.OnConnect_PreUnityGame);
        else
            ConnectRHB();
    }
    */

    public void CalibrateRHB(UnityAction onComplete = null)
    {
        for (int i = 0; i < MAX_ATTEMPTS; i++)
            if (distalRobot.Calibration(DistalComm.CalibrationType.AxisCalib)) break;

        // NOTE: reinstated this routine after adding offset (OFFSET_CALIB_RADIAL) to the reference RADIAL positions (20.09.2025):
        for (int i = 0; i < MAX_ATTEMPTS; i++)
            if (distalRobot.Calibration(DistalComm.CalibrationType.AllForceSensorsZeroCalib)) break;

        // Engage brakes:
        bool success = SetBrakesRHB(ENGAGE_BRAKE, ENGAGE_BRAKE, N_ATTEMPTS_BRAKE, DELAY_BRAKE_MSEC);

        // Display section:
        if (DISP_CONSOLE_ON)
            ExternalConsoleLogger.Log("CalibrateRHB((): ENGAGE BRAKES success = [" + success + "]\n");

        onComplete.Invoke();
    }

    public void OnCalibrate_CmdStartExercise()
    {
        const float CALL_DELAY = 0.1f;

        //////////////////////////////////////////////////////////////////
        // Call loader:
        //////////////////////////////////////////////////////////////////

        DOVirtual.DelayedCall(CALL_DELAY, () =>
        {
            // isExerciseStarted = false;
            isCalibrated = true;

            if (USE_STANDALONE_UI)
                BikeGameUI.instance.SetLoaderState(false);

            SceneManager.LoadScene(PrototypeSceneName);
        });
    }

    private void SetGainRHB(float radialGain, float angularGain)
    {
        for (int i = 0; i < MAX_ATTEMPTS; i++)
            if (distalRobot.SetGain(radialGain, angularGain))
                break;
    }

    public void SetOffsetForcesRHB(float radialOffsetForce, float angularOffsetForce)
    {
        for (int i = 0; i < MAX_ATTEMPTS; i++)
            if (distalRobot.SetOffsetForces(radialOffsetForce, angularOffsetForce))
                break;
    }

    public bool SetBrakesRHB(bool unlockRadial, bool unlockRotational, int N_attempt, int delay_msec)
    {
        bool success = false;

        for (int i = 0; i < N_attempt; i++)
        {
            success = distalRobot.ControlBrakes(unlockRadial, unlockRotational);

            if (success)
                break;

            Task.Delay(delay_msec);
        }

        return success;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // Real-time loop - CRITICAL:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    private void RealTimeControlLoop()
    {
        ////////////////////////////////////////////////////////////////////////////
        // Motion routine (to baseline position / to home position) vars:
        ////////////////////////////////////////////////////////////////////////////

        const int N_STEPS_MOTION_ROUTINE = 80;

        bool motionRoutineActiveBaseline = false;
        bool motionRoutineActiveHome = false;

        float pos_radial_routine_end = POS_RADIAL_MIN_OFFS;
        float factor_blend;

        ////////////////////////////////////////////////////////////////////////////
        // Display variables:
        ////////////////////////////////////////////////////////////////////////////

        const int DECIM_DISP = 100;

        if (DISP_RT_LOOP_ON)
            ExternalConsoleLogger.Log("RealTimeControlLoop(): STARTING \n");

        ////////////////////////////////////////////////////////////////////////////
        // Safety catch: wait for class instances to build:
        ////////////////////////////////////////////////////////////////////////////

        while (MotorbikeController.instance == null || Track.instance == null)
            Task.Delay(10);

        ////////////////////////////////////////////////////////////////////////////
        // Enforce "bike upright" constraint - CRITICAL: 
        ////////////////////////////////////////////////////////////////////////////
        UnityMainThreadDispatcher.Instance().Enqueue(() => MotorbikeController.instance.uprightConstraintEnforce(ref UPRIGHT_CONSTR_ON)); // constraint flag (13.09.2025) 

        ////////////////////////////////////////////////////////////////////////////
        // Real-time loop:
        ////////////////////////////////////////////////////////////////////////////

        while (enabledControlThread)
        {
            ////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////
            // Thread sleep & time elapsed computation:
            ////////////////////////////////////////////////////////////////////////////   
            //////////////////////////////////////////////////////////////////////////// 

            if (!USE_RT_TIMER_LOCK)
            {
                // Unconditional thread sleep:
                Thread.Sleep(DT_STEP_APP_MSEC);
            }
            else
            {
                // Conditional thread sleep:
                timerLocked = true;

                while (timerLocked)
                {
                    Thread.Sleep(DT_STEP_APP_MSEC);
                    timerLocked = false;
                }
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

            // Display section:
            if (DISP_RT_LOOP_ON && (step_count % DECIM_DISP == 0 || runProceduresExerciseStart || runProceduresExerciseStop))
            {
                ExternalConsoleLogger.Log("________________________________________________________");
                ExternalConsoleLogger.Log("RealTimeControlLoop(): step_count = [" + step_count + "]");

                if (runProceduresExerciseStart)
                    ExternalConsoleLogger.Log("    runProceduresExerciseStart = [" + runProceduresExerciseStart + "]\n");

                if (runProceduresExerciseStop)
                    ExternalConsoleLogger.Log("    runProceduresExerciseStop = [" + runProceduresExerciseStop + "]\n");
            }

            ////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////
            // Game-specific code:
            //////////////////////////////////////////////////////////////////////////// 
            ////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////
            // Exercise START procedures:
            //////////////////////////////////////////////////////////////////////////// 

            if (runProceduresExerciseStart)
            {
                runProceduresExerciseStart = false; // reset flag - CRITICAL

                //////////////////////////////////////////////////////////////////
                // Motion to BASELINE point - activate:
                //////////////////////////////////////////////////////////////////

                motionRoutineActiveBaseline = true;

                //////////////////////////////////////////////////////////////////
                // Start step counter:
                //////////////////////////////////////////////////////////////////

                step_count = 0;

                //////////////////////////////////////////////////////////////////
                // PERFORMANCE metrics - set up variables: 
                //////////////////////////////////////////////////////////////////

                // Start UNDERSTEER event counter
                MotorbikeController.instance.step_count_understeer = 0;

                // Start FALL counter:
                MotorbikeController.instance.step_count_fall = 0;
                MotorbikeController.instance.bike_fallen_prev = false;

                // Distance traveled during exercise:
                MotorbikeController.instance.dist_traveled = 0f;
            }

            ////////////////////////////////////////////////////////////////////////////
            // Exercise STOP procedures:
            //////////////////////////////////////////////////////////////////////////// 

            else if (runProceduresExerciseStop)
            {
                runProceduresExerciseStop = false; // reset flag - CRITICAL

                //////////////////////////////////////////////////////////////////
                // Motion to HOME point - activate:
                //////////////////////////////////////////////////////////////////

                motionRoutineActiveHome = true;

                //////////////////////////////////////////////////////////////////
                // Start step counter:
                //////////////////////////////////////////////////////////////////

                step_count = 0;

                //////////////////////////////////////////////////////////////////
                // PERFORMANCE metrics: compute UNDERSTEER fraction
                //////////////////////////////////////////////////////////////////

                frac_understeer = (float)MotorbikeController.instance.step_count_understeer / step_count;

                /////////////////////////////////////////////////////////
                // Set DataManager 'race started' flag (27.08.2025):
                /////////////////////////////////////////////////////////

                DataManager.instance.isRaceStarted = false;

                /////////////////////////////////////////////////////////
                // Go back to menu and restart game
                // NOTE: proper game restart required for Care Platform
                /////////////////////////////////////////////////////////

                STATE_PREGAME = ST_SELECT_BIKE_TYPE;

                BikeGameUI.instance.OnConnect_PreUnityGame();
            }

            //////////////////////////////////////////////////////////////////////////// 
            ////////////////////////////////////////////////////////////////////////////
            // RHB motion cases:
            //////////////////////////////////////////////////////////////////////////// 
            //////////////////////////////////////////////////////////////////////////// 

            if (isExerciseStarted || motionRoutineActiveHome)
            {

                ////////////////////////////////////////////////////////////////////////////
                // Perform exercise:
                //////////////////////////////////////////////////////////////////////////// 

                if (!motionRoutineActiveBaseline && !motionRoutineActiveHome)
                {
                    ////////////////////////////////////////////////////////////////////////////
                    // RHB coordinates:
                    ////////////////////////////////////////////////////////////////////////////

                    float pos_radial = distal_data.PositionR;

                    float pos_rot = distal_data.PositionP;
                    float dt_pos_rot = distal_data.VelocityP;

                    ////////////////////////////////////////////////////////////////////////////
                    // Retrieve data from bike and track objects:
                    ////////////////////////////////////////////////////////////////////////////

                    // Retrieve bike pose coordinates:
                    angle_roll_bike = MotorbikeController.instance.bike_pose_data.angle_roll_bike;
                    dt_angle_roll_bike = MotorbikeController.instance.bike_pose_data.dt_angle_roll_bike;

                    // Retrieve fedback control data:
                    angle_roll_targ = MotorbikeController.instance.fbk_ctrl_data.angle_roll_targ;
                    dt_angle_roll_targ = MotorbikeController.instance.fbk_ctrl_data.dt_angle_roll_targ;

                    ////////////////////////////////////////////////////////////////////////////
                    // Set Target commands for steering and throttle - CRITICAL:
                    //////////////////////////////////////////////////////////////////////////// 

                    CmdSetTargetSteerAndThrottleCases(
                        pos_rot, dt_pos_rot, // was dt_pos_rot_kal (24.09.2025)
                        angle_roll_targ, angle_roll_bike,
                        dt_angle_roll_targ, dt_angle_roll_bike,
                        CASE_CTRL_MODE);
                }

                //////////////////////////////////////////////////////////////////
                // Perform motion to patient's BASELINE (zero throttle) or HOME position:
                //////////////////////////////////////////////////////////////////

                else if (motionRoutineActiveBaseline)
                {
                    // Blend factor for gains & stiffness values:
                    factor_blend = (float)step_count / N_STEPS_MOTION_ROUTINE;

                    // Motion step:
                    MotionRoutineRHBSimpleStep(POS_RADIAL_THROT_ZERO_OFFS,
                        factor_blend);

                    // Motion completion procedures:
                    if (step_count == N_STEPS_MOTION_ROUTINE)
                    {
                        // Reset motion routine flags:
                        motionRoutineActiveBaseline = false;

                        // Set force feedback gains - CRITICAL:
                        SetForceFeebackGainCases(CASE_CTRL_MODE);

                        // Display section:
                        if (DISP_CONSOLE_ON)
                        {
                            ExternalConsoleLogger.Log("RealTimeControlLoop(): COMPLETED motionRoutineActiveBaseline \n");
                        }
                    }
                }

                //////////////////////////////////////////////////////////////////
                // Perform motion to patient's HOME position:
                //////////////////////////////////////////////////////////////////

                else if (motionRoutineActiveHome)
                {
                    // Blend factor for gains & stiffness values:
                    factor_blend = (float)step_count / N_STEPS_MOTION_ROUTINE;

                    // Motion step:
                    MotionRoutineRHBSimpleStep(POS_RADIAL_MIN_OFFS,
                        factor_blend);

                    // Motion completion procedures:
                    if (step_count == N_STEPS_MOTION_ROUTINE)
                    {
                        // Reset motion routine flags:
                        motionRoutineActiveHome = false;

                        // Engage brakes:
                        // bool success = SetBrakesRHB(ENGAGE_BRAKE, ENGAGE_BRAKE, N_ATTEMPTS_BRAKE, DELAY_BRAKE_MSEC);

                        // Display section:
                        if (DISP_CONSOLE_ON)
                        {
                            // ExternalConsoleLogger.Log("RealTimeControlLoop(): motionRoutineActiveHome ENGAGE BRAKES success = [" + success + "]\n");
                            ExternalConsoleLogger.Log("RealTimeControlLoop(): COMPLETED motionRoutineActiveHome \n");
                        }
                    }
                }
            }

            // Display section:
            if (DISP_RT_LOOP_ON && step_count % DECIM_DISP == 0)
                ExternalConsoleLogger.Log("    Exercise active [" + isExerciseStarted + "]\n");

            ////////////////////////////////////////////////////////////////////////////
            // Update step counter - CRITICAL:
            ////////////////////////////////////////////////////////////////////////////

            step_count++;

        } // while (enabledControlThread)
    }

    ////////////////////////////////////////////////////////////////////////////
    // Ancillary functions - RHB control:
    ////////////////////////////////////////////////////////////////////////////

    public bool ToggleExerciseRHB(bool is_exerc_started)
    {
        bool is_exerc_started_new;

        //////////////////////////////////////////////////////////////////
        // Start exercise:
        //////////////////////////////////////////////////////////////////

        if (!is_exerc_started)
        {
            Debug.Log("is exercise started: " + is_exerc_started);
            is_exerc_started_new = StartExerciseRHB();

            // Display section:
            if (DISP_CONSOLE_ON)
                ExternalConsoleLogger.Log("ToggleExerciseRHB() / StartExerciseRHB(): isExerciseStartedNew = [" + is_exerc_started_new + "] \n");

            // Disengage brakes:
            bool success = SetBrakesRHB(DISENGAGE_BRAKE, DISENGAGE_BRAKE, N_ATTEMPTS_BRAKE, DELAY_BRAKE_MSEC);

            // Display section:
            if (DISP_CONSOLE_ON)
                ExternalConsoleLogger.Log("ToggleExerciseRHB() / StartExerciseRHB(): DISENGAGE BRAKES success = [" + success + "]\n");
        }

        //////////////////////////////////////////////////////////////////
        // Stop exercise:
        //////////////////////////////////////////////////////////////////

        else
        {
            Debug.Log("is exercise started: " + is_exerc_started);

            is_exerc_started_new = StopExerciseRHB(() => { });
            Debug.Log("is exercise started new: " + is_exerc_started_new);

            // Display section:
            if (DISP_CONSOLE_ON)
                ExternalConsoleLogger.Log("ToggleExerciseRHB() / StopExerciseRHB(): isExerciseStartedNew = [" + is_exerc_started_new + "] \n");
        }

        return is_exerc_started_new;
    }

    private bool StartExerciseRHB(UnityAction onComplete = null) // bool unlockRadial, bool unlockRotational
    {
        bool is_exerc_started;
        bool startExerciseSucess;

        const bool OVERRIDE_START_EXERC = false;

        const int N_ATTEMPTS_MAX = 50;
        const int DELAY_START_EXERC_MSEC = 50;

        /*
        if (isExerciseStarted)
        {
            if (DISP_CONSOLE_ON)
            {
                ExternalConsoleLogger.Log(" ");
                ExternalConsoleLogger.Log("StartExercise(): isExerciseStarted ALREADY \n");
            }

            onComplete?.Invoke();

            if (isCalibrated)
                OnExerciseStart?.Invoke();

            return;
        }
        */

        //////////////////////////////////////////////////////////////////
        // Send Start Exercise command to firmware:
        //////////////////////////////////////////////////////////////////

        DistalComm.CommandedBrakes BRAKES_INIT = new()
        {
            R = DistalComm.BrakeCommand.Disengage,
            P = DistalComm.BrakeCommand.Disengage
        };

        DistalComm.OffsetForceTorque FORCE_OFFS_INIT = new()
        {
            OffsetRForce = 0,
            OffsetPTorque = 0
        };

        if (OVERRIDE_START_EXERC)
            startExerciseSucess = true;
        else
        {
            int att_count = 0;
            do
            {
                startExerciseSucess = distalRobot.StartExercise(1, BRAKES_INIT, FORCE_OFFS_INIT);
                Thread.Sleep(DELAY_START_EXERC_MSEC);
            }
            while (++att_count <= N_ATTEMPTS_MAX && !startExerciseSucess);
        }

        //////////////////////////////////////////////////////////////////
        // Start Exercise procedures:
        //////////////////////////////////////////////////////////////////

        if (startExerciseSucess)
        {
            is_exerc_started = true;
            runProceduresExerciseStart = true;

            //////////////////////////////////////////////////////////////////
            // Start timer:
            //////////////////////////////////////////////////////////////////

            if (USE_RT_TIMER_LOCK)
            {
                timerLocked = true;
                timerActivePrev = timerActive;
                timerActive = true;
                Thread.Sleep(DT_STEP_APP_MSEC);
                timerLocked = false;
            }

            //////////////////////////////////////////////////////////////////
            // Initiate recording on new data file - can't remove this without missing the call (07.10.2025):
            //////////////////////////////////////////////////////////////////

            DataManager.instance.SetupRecordingEvents();

            //////////////////////////////////////////////////////////////////
            // Display section:
            //////////////////////////////////////////////////////////////////

            if (DISP_CONSOLE_ON)
            {
                ExternalConsoleLogger.Log(" ");
                ExternalConsoleLogger.Log("StartExerciseRHB(): SUCCESS runProceduresExerciseStart [" + runProceduresExerciseStart + "]\n"); // timerActivePrev, timerActive
            }

            if (isCalibrated)
                OnExerciseStart?.Invoke();

            onComplete?.Invoke();
        }
        else
        {
            is_exerc_started = false;
            runProceduresExerciseStart = false;

            // Display section:
            if (DISP_CONSOLE_ON)
            {
                ExternalConsoleLogger.Log(" ");
                ExternalConsoleLogger.Log("StartExerciseRHB(): FAIL runProceduresExerciseStart [" + runProceduresExerciseStart + "]\n");
            }
        }

        return is_exerc_started;
    }

    private bool StopExerciseRHB(UnityAction onComplete = null)
    {
        bool is_exerc_started;

        /*
        if (!isExerciseStarted)
        {
            if (DISP_CONSOLE_ON)
            {
                ExternalConsoleLogger.Log(" ");
                ExternalConsoleLogger.Log("....................................................................");
                ExternalConsoleLogger.Log("StopExercise(): !isExerciseStarted  - return \n");
            }

            OnExerciseStop?.Invoke();
            onComplete?.Invoke();
            return;
        }
        */

        if (USE_STANDALONE_UI)
        {
            BikeGameUI.instance.SetLoaderText("Stopping Exercise...");
            BikeGameUI.instance.SetLoaderState(true);
        }

        Time.timeScale = 0f;
        DOTween.PauseAll();

        /////////////////////////////////////////////////////////
        // Stop timer:
        /////////////////////////////////////////////////////////

        if (USE_RT_TIMER_LOCK)
        {
            timerLocked = true;
            timerActivePrev = timerActive;
            timerActive = false;
            Thread.Sleep(DT_STEP_APP_MSEC);
            timerLocked = false;
        }

        //////////////////////////////////////////////////////////////////
        // NOTE: game-specific procedures removed 07.10.2025:
        //////////////////////////////////////////////////////////////////

        is_exerc_started = false;
        runProceduresExerciseStop = true;

        // isCalibrated = false; // removed to avoid missed Enter commands

        // Display section:
        if (DISP_CONSOLE_ON)
        {
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("StopExerciseRHB(): SUCCESS runProceduresExerciseStop [" + runProceduresExerciseStop + "]\n");
        }

        OnExerciseStop?.Invoke();
        onComplete?.Invoke();

        /////////////////////////////////////////////////////////
        // Go back to menu and restart game
        /////////////////////////////////////////////////////////

        SceneManager.LoadScene(0);
        Time.timeScale = 1f;
        DOTween.PlayAll();

        return is_exerc_started;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Force feedback gains - CASES:
    ////////////////////////////////////////////////////////////////////////////

    void SetForceFeebackGainCases(int case_ctrl_mode)
    {
        switch (case_ctrl_mode)
        {
            case CTRL_ASSISTED:

                SetGainRHB(
                    FORCE_GAIN_RADIAL,
                    FORCE_GAIN_ROT);
                break;

            case CTRL_AUTO_STEER_AUTO_THROT:
            case CTRL_AUTO_STEER_MANUAL_THROT:

                SetGainRHB(0f, 0f);
                break;

            case CTRL_MANUAL_SIMPLE:

                SetGainRHB(
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

    void CmdSetTargetSteerAndThrottleCases(
        float pos_rot_est, float dt_pos_rot_est,
        float angle_roll_targ_this, float angle_roll_bike_this,
        float dt_angle_roll_targ_this, float dt_angle_roll_bike_this,
        int case_ctrl_mode)
    {

        switch (case_ctrl_mode)
        {
            case CTRL_ASSISTED:

                // Radial stiffness (throttle):
                float k_stiff_radial_throt;

                if (FACT_ASSIST_THROTTLE > 0f)
                    k_stiff_radial_throt = K_STIFF_RADIAL_THROT_AUTO;
                else
                    k_stiff_radial_throt = K_STIFF_RADIAL_THROT_MANUAL;

                // Reference equilibrium position:
                pos_rot_eq_ref = FRAC_POS_ROT_INPUT_PATIENT * angle_roll_targ_this;
                dt_pos_rot_eq_ref = FRAC_POS_ROT_INPUT_PATIENT * dt_angle_roll_targ_this;

                ///////////////////////////////////////////////////////////////////////////
                // Compute assistive torque:
                ///////////////////////////////////////////////////////////////////////////

                // Formula updated after updating offset force cmd in firmware (24.09.2025):
                float TORQUE_ASSIST_LIM = FACT_ASSIST_STEER * TORQUE_ASSIST_STEER_MAX;

                // Angle-depenedent stiffness:
                float K_STIFF_ROT_ASSIST = FRAC_ASSIST_STIFF * K_STIFF_ROT_TRACKING;

                // NOTE: adding a damping term using dt_pos_rot_est may introduce quantization noise:
                float torque_assist_raw =
                    K_STIFF_ROT_ASSIST * (pos_rot_eq_ref - pos_rot_est);

                // Assistive torque - apply limits:
                torque_assist = Math.Clamp(torque_assist_raw, -TORQUE_ASSIST_LIM, TORQUE_ASSIST_LIM);

                // Set target command for baseline state
                // (1) Generates physical rotation limits
                // (2) Provides bias rotation if needed
                CmdSetTarget_FeedbackCtrl_WithLimit(
                    pos_rot_est, 0f,
                    0f, k_stiff_radial_throt, B_DAMP_ROT_TRACKING,
                    K_STIFF_ROT_BASE);

                // Command offset forces for ASSISTANCE - CRITICAL:
                // SetOffsetForces(0f, torque_assist); // changed from this to avoid stacking messages (25.09.2025)
                distalRobot.SetOffsetForces(0f, torque_assist);
                break;

            case CTRL_AUTO_STEER_AUTO_THROT:
            case CTRL_AUTO_STEER_MANUAL_THROT:

                // Rotational stiffness (steering):
                if (case_ctrl_mode == CTRL_AUTO_STEER_AUTO_THROT)
                    k_stiff_radial_throt = K_STIFF_RADIAL_THROT_AUTO;
                else
                    k_stiff_radial_throt = K_STIFF_RADIAL_THROT_MANUAL;

                // Reference equilibrium position:
                pos_rot_eq_ref = FRAC_POS_ROT_INPUT_PATIENT * angle_roll_bike_this;

                CmdSetTarget_AutoSteer(
                    pos_rot_eq_ref,
                    K_STIFF_ROT_TRACKING,
                    k_stiff_radial_throt);
                break;

            case CTRL_MANUAL_SIMPLE:

                CmdSetTarget_FeedbackCtrl_WithLimit(
                    pos_rot_est, 0f,
                    0f, K_STIFF_RADIAL_THROT_MANUAL, 0f,
                    K_STIFF_ROT_BASE);
                break;

            default: // no command
                break;
        }
    }

    private void CmdSetTarget_AutoSteer(float pos_rot_eq_ref, float k_stiff_rot_steer, float k_stiff_radial_throt)
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

        if (isExerciseStarted)
            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_THROT_ZERO_OFFS, pos_rot_eq_ref,
                k_stiff_radial_throt, k_stiff_rot_steer,
                B_DAMP_RADIAL_BASE, b_damp_rot_steer,
                SWITCH_RADIAL, SWITCH_ROT);
        else
            success_set_target = false;
    }

    private void CmdSetTarget_FeedbackCtrl_WithLimit(
        float pos_rot, float pos_rot_eq_ref,
        float k_stiff_rot_ref, float k_stiff_radial_throt, float b_damp_rot_equiv,
        float k_stiff_rot_base)
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

        ////////////////////////////////////////////////////////////////////////////
        // Compute equivalent impedance parameters:
        ////////////////////////////////////////////////////////////////////////////

        float pos_rot_eq_lim;
        float k_stiff_rot_lim;

        if (pos_rot > ANGLE_ROT_LIM)
        {
            pos_rot_eq_lim = ANGLE_ROT_LIM;
            k_stiff_rot_lim = K_STIFF_ROT_LIM;
        }

        else if (pos_rot < -ANGLE_ROT_LIM)
        {
            pos_rot_eq_lim = -ANGLE_ROT_LIM;
            k_stiff_rot_lim = K_STIFF_ROT_LIM;
        }

        else
        {
            pos_rot_eq_lim = 0f; // dummy value
            k_stiff_rot_lim = 0f; // true value
        }

        k_stiff_rot_equiv = k_stiff_rot_ref + k_stiff_rot_lim + k_stiff_rot_base;

        pos_eq_rot_equiv =
            (k_stiff_rot_ref * pos_rot_eq_ref + k_stiff_rot_lim * pos_rot_eq_lim)
             / k_stiff_rot_equiv;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        // TODO: consider change to plain distalRobot.SetTarget to reduce overhead (possible risk is timeouts):
        if (isExerciseStarted)
            success_set_target = distalRobot.HL_SetTarget(
                IDX_TARG_BASE,
                POS_RADIAL_THROT_ZERO_OFFS, pos_eq_rot_equiv,
                k_stiff_radial_throt, k_stiff_rot_equiv,
                B_DAMP_RADIAL_BASE, b_damp_rot_equiv,
                SWITCH_RADIAL, SWITCH_ROT);
        else
            success_set_target = false;
    }

    ////////////////////////////////////////////////////////////////////////////
    // GAME LEVEL & ASSIST FACTOR update (29.09.2025):
    ////////////////////////////////////////////////////////////////////////////

    public int PerformGameLevelChange(
        int game_level_curr, int num_game_levels, ref int level_change,
        float frac_underst,
        int step_count_fall,
        float dist_traveled, float length_track)
    {
        if (dist_traveled < FRAC_LENGTH_TRACK_LEGIT_RACE * length_track)
            game_level_change = 0;
        else if (frac_underst <= FRAC_UNDERSTEER_LEVEL_UP_MAX && step_count_fall == 0)
            level_change = 1;
        else if (frac_underst >= FRAC_UNDERSTEER_LEVEL_DOWN_MIN || step_count_fall > N_FALLS_LIM)
            level_change = -1;
        else
            level_change = 0;

        // Validate game level change:
        if ((game_level_curr == num_game_levels && level_change > 0) ||
            (game_level_curr == 1 && level_change < 0))
            level_change = 0;

        return game_level_curr + level_change;
    }

    public float FactorAssistCalc(int game_level)
    {
        float fact_assist_steer;

        // Slopes for piecewise computation:
        float m1 = (1f - FACT_ASSIST_MID) / GAME_LEVEL_MID;
        float m2 = FACT_ASSIST_MID / GAME_LEVEL_MID;

        if (game_level <= GAME_LEVEL_MID)
            fact_assist_steer = 1f - m1 * (float)game_level;
        else
            fact_assist_steer = m2 * (N_GAME_LEVELS - (float)game_level);

        return fact_assist_steer;
    }

    private bool SetTargetValidated(byte targetIndex,
        float radialValue, float rotationValue,
        float radialStiffness, float rotationStiffness,
        float radialDamping, float rotationDamping,
        float radialGain, float rotationGain, UnityAction onComplete = null)
    {
        bool success = false;

        radialValue = Mathf.Clamp(radialValue, POS_RADIAL_MIN_OFFS, POS_RADIAL_MAX);
        rotationValue = Mathf.Clamp(rotationValue, POS_ROT_MIN, POS_ROT_MAX);

        for (int i = 0; i < MAX_ATTEMPTS; i++)
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
            POS_RADIAL_MIN_OFFS, 0f,
            0f, 0f,
            0f, 0f,
            0f, 0f);
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // Functions to replace MoveDistalRoutine() and RotateDistalRoutine():
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    private bool MotionRoutineRHBSimple(float pos_rad_targ, int N_steps, int Dt_motion_msec)
    {
        bool success_all = true;
        bool success_step;

        for (int step_count = 0; step_count <= N_steps; step_count++)
        {
            float factor_blend = (float)step_count / N_steps;

            success_step = MotionRoutineRHBSimpleStep(pos_rad_targ, factor_blend);

            if (success_all && !success_step)
                success_all = false;

            Thread.Sleep(Dt_motion_msec / N_steps);
        }

        return success_all;
    }

    private bool MotionRoutineRHBSimpleStep(float pos_rad_targ, float factor_blend)
    {
        bool success_step = SetTargetValidated(
            IDX_TARG_BASE,
            pos_rad_targ, POS_ROT_BASE,
            factor_blend * K_STIFF_RADIAL_WALL, factor_blend * K_STIFF_ROT_WALL,
            B_DAMP_RADIAL_WALL, B_DAMP_ROT_WALL,
            factor_blend, factor_blend);

        return success_step;
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // Update() function: ONLY for standalone game UI (13.10.2025)
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    private void Update()
    {
        if (USE_STANDALONE_UI)
        {
            ////////////////////////////////////////////////////////////////////////////
            // Initialize Unity game & launch exercise (19.09.2025):
            // This function
            //      (1) Generates the game settings when UNITY_GAME is used, and
            //      (2) Toggles the Exercise (i.e. STARTS / STOPS the exercise)
            // 
            // InitUnityGame_StartExercise() must be replaced by a different function when CARE_PLATFORM is used 
            // NOTE: both functions must set the "bike upright" constraint (UPRIGHT_CONSTR_ON) to true before the exercise is launched:
            ////////////////////////////////////////////////////////////////////////////

            BikeGameUI.instance.InitUnityGame_StartExercise(
                ref USE_BEGINNER_BIKE_CONSTR,
                ref CASE_CTRL_MODE,
                ref game_level_curr,
                ref FACT_ASSIST_STEER,
                ref FACT_ASSIST_THROTTLE,
                ref FRAC_POS_ROT_INPUT_PATIENT,
                ref POS_RADIAL_THROT_ZERO,
                ref K_STIFF_RADIAL_THROT_MANUAL,
                ref SPEED_AUTO_THROTTLE_MAX_KPH,
                ref RACE_DIRECTION,
                ref UPRIGHT_CONSTR_ON
            );

            // Offset the RADIAL reference positions to account for initial calibration errors - CRITICAL (20.09.2025):
            POS_RADIAL_THROT_ZERO_OFFS =
                POS_RADIAL_THROT_ZERO +
                OFFS_POS_RADIAL_CALIB;

            POS_RADIAL_MIN_OFFS =
                POS_RADIAL_MIN +
                2.0f * OFFS_POS_RADIAL_CALIB;
        }

        ////////////////////////////////////////////////////////////////////////////
        // Launch real-time control thread - CRITICAL:
        ////////////////////////////////////////////////////////////////////////////

        if (!enabledControlThread)
        {
            enabledControlThread = true;

            rtControlThread?.Abort();
            rtControlThread = new Thread(RealTimeControlLoop);
            rtControlThread.Start();

            // Display section:
            if (DISP_CONSOLE_ON)
            {
                ExternalConsoleLogger.Log("RHBCtrlBike() / Update(): rtControlThread START \n");
            }
        }
    }
}