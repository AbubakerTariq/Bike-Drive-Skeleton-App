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
    // CARE_PLATFORM controlled parameters - Patient-dependent game parameters:
    //////////////////////////////////////////////////////////////////////////// 

    // Assistance factor (between 0 and 1.0)
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM game level)
    public float FACT_ASSIST_STEER = 0f;

    public const int THROTTLE_MODE_MANUAL = 0;
    public const int THROTTLE_MODE_AUTO = 1;

    // Throttle mode (0: MANUAL, 1: AUTO, default is AUTO)
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM throttle setting)
    public float FACT_ASSIST_THROTTLE = (float)THROTTLE_MODE_AUTO;

    // Scaling factor for Patient's rotational inputs
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient ROM data)
    public float FRAC_POS_ROT_INPUT_PATIENT = 0.4f;

    // Maximum assistive torque:
    const float TORQUE_ASSIST_MAX = 0.2f;

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - MANUAL throttle parameters:
    //////////////////////////////////////////////////////////////////////////// 

    // Handles opening distance (meters) for zero throttle input:
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient hand opening data; keep default for first CARE_PLATFORM release)
    public float POS_RADIAL_THROT_ZERO = 0.029f;

    // Throttle stiffness for MANUAL throttle mode:
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient stiffness calibration data; keep default for first CARE_PLATFORM release)
    private float K_STIFF_RADIAL_THROT_MANUAL = 2500f;

    // Offset of the THROTTLE zero position to account for initial calibration errors (PD discussion on 20.09.2025)
    public float OFFSET_CALIB_RADIAL = 0.0005f;
    public float POS_RADIAL_THROT_ZERO_OFFS;

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - AUTO throttle parameters:
    //////////////////////////////////////////////////////////////////////////// 

    // AUTO throttle speed limit in kph:
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM game level)
    public float SPEED_AUTO_THROTTLE_MAX_KPH = 125f;

    ////////////////////////////////////////////////////////////////////////////
    // UNITY_GAME: states for PRE-GAME procedures:
    //////////////////////////////////////////////////////////////////////////// 

    private const int ST_SELECT_BIKE_TYPE = 1;
    private const int ST_SET_CTRL_MODE = 2;
    private const int ST_SET_FACT_ASSIST_STEER = 3;
    private const int ST_SET_FACT_ASSIST_THROTTLE = 4;
    private const int ST_CALIBRATE = 5;

    private int STATE_PREGAME = ST_SELECT_BIKE_TYPE; // initial state for UNITY_GAME procedures

    ////////////////////////////////////////////////////////////////////////////
    // SetExercise() parameters:
    //////////////////////////////////////////////////////////////////////////// 

    private float OFFS_FORCE_RADIAL_INIT = 0f;
    private float OFFS_TORQUE_ROT_INIT = 0f;

    private bool SAFETY_TCP_APP_ON = false;
    private bool STABILITY_SET_TARG_ON = true;

    private const bool ENGAGE_BRAKE = false;
    private const bool DISENGAGE_BRAKE = true;

    ////////////////////////////////////////////////////////////////////////////
    // Target indices:
    ////////////////////////////////////////////////////////////////////////////

    public const int NUM_TARGETS = 1;
    private byte IDX_TARG_BASE = 1;

    ////////////////////////////////////////////////////////////////////////////
    // Object instances:
    ////////////////////////////////////////////////////////////////////////////

    public static ReHandyBotController instance;
    private DistalComm distalRobot = new(); // Distal Control Library object

    ////////////////////////////////////////////////////////////////////////////
    // RHB control settings - CRITICAL
    // NOTE: use [RHB ctrl params - stability v5b game settings 4-axis.xlsx] to calculate damping as a function of stiffness
    ////////////////////////////////////////////////////////////////////////////

    private float FORCE_GAIN_RADIAL = 9.0f;
    private float FORCE_GAIN_ROT = 14.0f;

    private float K_STIFF_RADIAL_WALL = 2500f; // use with zero feedback gain
    private float B_DAMP_RADIAL_WALL = 0f; // rely on embedded HL_SetTarget stability

    private float K_STIFF_ROT_WALL = 1.2f; // use with zero feedback gain
    private float B_DAMP_ROT_WALL = 0f; // rely on embedded HL_SetTarget stability

    private float POS_RADIAL_MIN = 0.0145f;
    private float POS_RADIAL_MAX = 0.06f;

    // Offset of the MINIMUM RADIAL position to account for initial calibration errors (PD discussion on 20.09.2025):
    private float POS_RADIAL_MIN_OFFS;

    private float POS_ROT_MIN = -Mathf.PI / 2f;
    private float POS_ROT_MAX = Mathf.PI / 2f;

    // Throttle - additional haptics settings:
    private float K_STIFF_RADIAL_THROT_AUTO = 5000f; // makes handles essentially rigid
    private float B_DAMP_RADIAL_BASE = 0f; // rely on embedded HL_SetTarget stability 

    // Steering - BASELINE haptics settings:
    private const float POS_ROT_BASE = 0f;

    private float K_STIFF_ROT_BASE = 0.1f; // 0.05f; // 
    private float B_DAMP_ROT_BASE = 0f; // rely on embedded HL_SetTarget stability

    // Stiffness for tracking control;
    private float K_STIFF_ROT_TRACKING = 1.6f; // 2.0f; // 
    private float B_DAMP_ROT_TRACKING = 0.03f; // 0.04f; // TODO: check stabilizing damping for K_STIFF_ROT_TRACKING + K_STIFF_ROT_BASE (use )

    ////////////////////////////////////////////////////////////////////////////
    // Impedance for RHB motion limits:
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
    // Kalman filter for rotation state estimation:
    ////////////////////////////////////////////////////////////////////////////

    // Values from Maxon motor simulation (including encoder quantization and gear ratio):
    const float ERR_EST_INIT = 0.1f;
    private float[] Q_PROC   = {9.43e-9f, 1.005e-4f};

    private float R_MEAS = 4.3e-6f; // 2.15e-6f; // was 2.15e-7

    KalmanFilter2D kal_f = new KalmanFilter2D(ERR_EST_INIT);

    ////////////////////////////////////////////////////////////////////////////
    // Rotation state estimates using Kalman filter:
    //////////////////////////////////////////////////////////////////////////// 

    public float pos_rot_kal;
    public float dt_pos_rot_kal;

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

    // Flag to maintain 'upright' constraint while exercise is inactive and throttle input is zero:
    public bool UPRIGHT_CONSTR_ON; // constraint flag (13.09.2025)

    ////////////////////////////////////////////////////////////////////////////
    // Constants:
    ////////////////////////////////////////////////////////////////////////////

    private const int MaxAttempts = 10;
    private const string ServerIP = "192.168.102.1";
    private const int ServerPort  = 3002;

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

    public Action OnExerciseStart;
    public Action OnExerciseStop;

    private const string PrototypeSceneName = "Prototype";

    ////////////////////////////////////////////////////////////////////////////
    // Control loop timers:
    ////////////////////////////////////////////////////////////////////////////

    private bool timerActive     = false;
    private bool timerActivePrev = false;
    private bool timerLocked     = false;

    public float timeElapsedValue = 0f;

    public int step_count = 0;

    ////////////////////////////////////////////////////////////////////////////
    // Thread and timer for SetTarget process:
    ////////////////////////////////////////////////////////////////////////////

    private System.Timers.Timer timerSetTarget;
    private Thread threadTimerSetTarget;

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
        // Stop all RHB related processes when the application is closed:
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
                    StartSystem(OnConnect_PreUnityGame);
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
            StartSystem(OnConnect_PreUnityGame);
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

    private void OnCalibrate_CmdStartExercise()
    {

        StartExercise(ENGAGE_BRAKE, ENGAGE_BRAKE, () =>
        {
            DOVirtual.DelayedCall(0.1f, () =>
            {
                loader.SetActive(false);
                pos_radial_min = distal_data.PositionR;
                pos_radial_min = Math.Clamp(pos_radial_min, POS_RADIAL_MIN_OFFS, POS_RADIAL_MAX);

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

    private void Calibrate(UnityAction onComplete = null)
    {
        for (int i = 0; i < MaxAttempts; i++)
            if (distalRobot.Calibration(DistalComm.CalibrationType.AxisCalib)) break;

        // NOTE: reinstated this routine after adding offset (OFFSET_CALIB_RADIAL) to the reference RADIAL positions (20.09.2025):
        for (int i = 0; i < MaxAttempts; i++)
            if (distalRobot.Calibration(DistalComm.CalibrationType.AllForceSensorsZeroCalib)) break;

        onComplete.Invoke();
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // UNITY_GAME: Pre-game actions:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    private void OnConnect_PreUnityGame()
    {
        SetBrakes(DISENGAGE_BRAKE, DISENGAGE_BRAKE);
        ExternalConsoleLogger.Log("        OnConnect(): SetBrakes(): cmd DISENGAGE \n");

        loader.SetActive(true);

        if (STATE_PREGAME == ST_SELECT_BIKE_TYPE)
        {
            loaderText.text =
               "CLICK on this screen and \n\n" +
               "Select BIKE TYPE: \n\n" +
               "PRO bike: hit [Enter] \n" +
               "Beginner: hit [B]";
        }

        loaderText.alignment = TextAlignmentOptions.MidlineLeft;
    }

    private void OnSelectGameSettings_PreUnityGame()
    {
        if (STATE_PREGAME == ST_SET_CTRL_MODE)
        {
            loaderText.text =
               "Select CONTROL MODE: \n\n" +
               "(1) ASSISTED CONTROL \n" +
               "(2) AUTO STEER / AUTO THROTTLE \n" +
               "(3) AUTO STEER / MANUAL THROTTLE \n" +
               "(4) PURE MANUAL";
        }

        else if (STATE_PREGAME == ST_SET_FACT_ASSIST_STEER) 
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
            string str_bike_type;
            string str_fact_assist;

            if (USE_BEGINNER_BIKE_CONSTR) 
                str_bike_type = "Bike type: BEGINNER";
            else
                str_bike_type = "Bike type: PRO" ;

            if (CASE_CTRL_MODE == CTRL_ASSISTED)
                str_fact_assist = 
                    "Assist factor STEERING = " + FACT_ASSIST_STEER + "\n" +                    
                    "Assist factor THROTTLE = " + FACT_ASSIST_THROTTLE;
            else
                str_fact_assist = " ";

            loaderText.text =
                str_bike_type +"\n\n" +
                "CONTROL MODE [" + CASE_CTRL_MODE + "]\n" +
                 str_fact_assist + "\n\n" +
                "Align grippers horizontally and close the grippers \n\n" +
                "Press Y to CALIBRATE";
        }
    }

    private void InitUnityGame_StartExercise(
        ref bool use_beginner_bike_constr,
        ref int case_ctrl_mode,
        ref float fact_assist_steer,
        ref float fact_assist_throttle,
        ref float frac_pos_rot_input_patient,
        ref float pos_radial_throt_zero,
        ref float k_stiff_radial_throt_manual,
        ref float speed_auto_throttle_max_kph,
        ref bool upright_constr_on)
    {

        ////////////////////////////////////////////////////////////////////////////
        // Fixed settings for UNITY_GAME:
        ////////////////////////////////////////////////////////////////////////////

        frac_pos_rot_input_patient = 0.4f;
        pos_radial_throt_zero = 0.029f;
        k_stiff_radial_throt_manual = 2500f;
        speed_auto_throttle_max_kph = 125f;

        ////////////////////////////////////////////////////////////////////////////
        // Selectable settings for UNITY_GAME:
        ////////////////////////////////////////////////////////////////////////////

        if (STATE_PREGAME != ST_CALIBRATE)

            SelectGameSettings_PreUnityGame(
                ref use_beginner_bike_constr,
                ref case_ctrl_mode,
                ref fact_assist_steer,
                ref fact_assist_throttle,
                OnSelectGameSettings_PreUnityGame);

        ////////////////////////////////////////////////////////////////////////////
        // Toggle Exercise state:
        ////////////////////////////////////////////////////////////////////////////

        else if (Input.GetKeyDown(KeyCode.Y))
        {
            Calibrate(OnCalibrate_CmdStartExercise);

            // Enforce "bike upright" constraint - CRITICAL: 
            MotorbikeController.instance.uprightConstraintEnforce(ref upright_constr_on); // constraint flag (13.09.2025) 

            // Display section:
            ExternalConsoleLogger.Log("_________________________________________________________________");
            ExternalConsoleLogger.Log("Update(): upright constraint [TRUE] \n");
        }

        ////////////////////////////////////////////////////////////////////////////
        // Toggle Exercise state:
        ////////////////////////////////////////////////////////////////////////////

        if (isCalibrated && Input.GetKeyDown(KeyCode.Return))
            ToggleExerciseState();
    }

    private void SelectGameSettings_PreUnityGame(
        ref bool use_beginner_bike_constr,
        ref int case_ctrl_mode,
        ref float fact_assist_steer,
        ref float fact_assist_throttle,
        UnityAction onComplete = null)
    {
        ////////////////////////////////////////////////
        // Select BIKE TYPE:
        ////////////////////////////////////////////////

        if (STATE_PREGAME == ST_SELECT_BIKE_TYPE)
        {
            if (Input.GetKeyDown(KeyCode.Return))
                use_beginner_bike_constr = false;
            else if (Input.GetKeyDown(KeyCode.B))
                use_beginner_bike_constr = true;
            else
                return;

            STATE_PREGAME = ST_SET_CTRL_MODE;

            onComplete.Invoke();
        }

        ////////////////////////////////////////////////
        // Select CONTROL MODE:
        ////////////////////////////////////////////////

        if (STATE_PREGAME == ST_SET_CTRL_MODE)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1))
                case_ctrl_mode = CTRL_ASSISTED;
            else if (Input.GetKeyDown(KeyCode.Alpha2))
                case_ctrl_mode = CTRL_AUTO_STEER_AUTO_THROT;
            else if (Input.GetKeyDown(KeyCode.Alpha3))
                case_ctrl_mode = CTRL_AUTO_STEER_MANUAL_THROT;
            else if (Input.GetKeyDown(KeyCode.Alpha4))
                case_ctrl_mode = CTRL_MANUAL_SIMPLE;
            else
                return;

            if (case_ctrl_mode == CTRL_ASSISTED)
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
                fact_assist_steer = 0f;
            else if (Input.GetKeyDown(KeyCode.Alpha1))
                fact_assist_steer = 1f;
            else if (Input.GetKeyDown(KeyCode.Alpha2))
                fact_assist_steer = 2f;
            else if (Input.GetKeyDown(KeyCode.Alpha3))
                fact_assist_steer = 3f;
            else if (Input.GetKeyDown(KeyCode.Alpha4))
                fact_assist_steer = 4f;
            else if (Input.GetKeyDown(KeyCode.Alpha5))
                fact_assist_steer = 5f;
            else if (Input.GetKeyDown(KeyCode.Alpha6))
                fact_assist_steer = 6f;
            else if (Input.GetKeyDown(KeyCode.Alpha7))
                fact_assist_steer = 7f;
            else if (Input.GetKeyDown(KeyCode.Alpha8))
                fact_assist_steer = 8f;
            else if (Input.GetKeyDown(KeyCode.Alpha9))
                fact_assist_steer = 9f;
            else
                return;

            // Convert assistance factor to fraction:
            fact_assist_steer /= 10f;

            STATE_PREGAME = ST_SET_FACT_ASSIST_THROTTLE;
            onComplete.Invoke();
        }

        ////////////////////////////////////////////////
        // Select STEERING assistance factor:
        ////////////////////////////////////////////////

        else if (STATE_PREGAME == ST_SET_FACT_ASSIST_THROTTLE)
        {
            if (Input.GetKeyDown(KeyCode.Alpha0))
                fact_assist_throttle = 0f;
            else if (Input.GetKeyDown(KeyCode.Alpha1))
                fact_assist_throttle = 1f;
            else
                return;

            STATE_PREGAME = ST_CALIBRATE;
            onComplete.Invoke();
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    // Real-time update - CRITICAL:
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    private void Update()
    {
        ////////////////////////////////////////////////////////////////////////////
        // Initialize Unity game & launch exercise (19.09.2025):
        // This function
        //      (1) Generates the game settings when UNITY_GAME is used, and
        //      (2) Toggles the Exercise (i.e. STARTS / STOPS the exercise)
        // 
        // InitUnityGame_StartExercise() must be replaced by a different function when CARE_PLATFORM is used 
        // NOTE: both functions must set th e"bike upright" constraint (UPRIGHT_CONSTR_ON) to true before the exercise is launched:
        ////////////////////////////////////////////////////////////////////////////

        InitUnityGame_StartExercise(
            ref USE_BEGINNER_BIKE_CONSTR,
            ref CASE_CTRL_MODE,
            ref FACT_ASSIST_STEER,
            ref FACT_ASSIST_THROTTLE,
            ref FRAC_POS_ROT_INPUT_PATIENT,
            ref POS_RADIAL_THROT_ZERO,
            ref K_STIFF_RADIAL_THROT_MANUAL,
            ref SPEED_AUTO_THROTTLE_MAX_KPH,
            ref UPRIGHT_CONSTR_ON
        );

        // Offset the RADIAL reference positions to account for initial calibration errors - CRITICAL (20.09.2025):
        POS_RADIAL_THROT_ZERO_OFFS = POS_RADIAL_THROT_ZERO + OFFSET_CALIB_RADIAL;
        POS_RADIAL_MIN_OFFS        = POS_RADIAL_MIN        + OFFSET_CALIB_RADIAL;

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
        float dt_pos_rot = ReHandyBotController.instance.distal_data.VelocityP;

        ////////////////////////////////////////////////////////////////////////////
        // Rotation state estimation using Kalman filter:
        //////////////////////////////////////////////////////////////////////////// 

        // Predict the next state:
        kal_f.Predict(Time.deltaTime, Q_PROC);

        // Update the filter with the new measurement:
        kal_f.Update(pos_rot, R_MEAS);

        // Get the estimated velocity and position:
        pos_rot_kal = kal_f.GetPositionEstimate();
        dt_pos_rot_kal = kal_f.GetVelocityEstimate();

        ////////////////////////////////////////////////////////////////////////////
        // Retrieve data from bike and track objects:
        ////////////////////////////////////////////////////////////////////////////

        if (MotorbikeController.instance != null && Track.instance != null)
        {
            // Retrieve bike pose coordinates:
            angle_roll_bike    = MotorbikeController.instance.bike_pose_data.angle_roll_bike;
            dt_angle_roll_bike = MotorbikeController.instance.bike_pose_data.dt_angle_roll_bike;

            // Retrieve fedback control data:
            angle_roll_targ    = MotorbikeController.instance.fbk_ctrl_data.angle_roll_targ;
            dt_angle_roll_targ = MotorbikeController.instance.fbk_ctrl_data.dt_angle_roll_targ;

            ////////////////////////////////////////////////////////////////////////////
            // Set Target commands for steering and throttle - CRITICAL:
            //////////////////////////////////////////////////////////////////////////// 

            if (ExerciseActive)
                CmdSetTargetSteerAndThrottleCases(
                    pos_rot, dt_pos_rot_kal, // was dt_pos_rot (20.09.2025)
                    angle_roll_targ, angle_roll_bike, 
                    dt_angle_roll_targ, dt_angle_roll_bike, 
                    CASE_CTRL_MODE);
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

    void CmdSetTargetSteerAndThrottleCases(
        float pos_rot, float dt_pos_rot, 
        float angle_roll_targ_this, float angle_roll_bike_this,
        float dt_angle_roll_targ_this, float dt_angle_roll_bike_this, 
        int case_ctrl_mode) {

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
                pos_rot_eq_ref    = FRAC_POS_ROT_INPUT_PATIENT * angle_roll_targ_this;
                dt_pos_rot_eq_ref = FRAC_POS_ROT_INPUT_PATIENT * dt_angle_roll_targ_this;
                
                // Compute assistive torque:
                float TORQUE_ASSIST_LIM = FACT_ASSIST_STEER * TORQUE_ASSIST_MAX;

                float torque_assist_raw = 
                    K_STIFF_ROT_TRACKING  * (pos_rot_eq_ref - pos_rot)
                    + B_DAMP_ROT_TRACKING * (dt_pos_rot_eq_ref - dt_pos_rot);                  

                // Assistive torque - apply limits:
                torque_assist = Math.Clamp(torque_assist_raw, -TORQUE_ASSIST_LIM, TORQUE_ASSIST_LIM);

                // Bias rotational stiffness bias to return rotation angle to zero :
                float k_stiff_rot_base_this = 0f;

                if (FACT_ASSIST_STEER < 0.2f)
                    k_stiff_rot_base_this = K_STIFF_ROT_BASE;
                else
                    k_stiff_rot_base_this = 0f;

                // Set target command for baseline state
                // (1) Generates physical rotation limits
                // (2) Provides bias rotation
                CmdSetTarget_FeedbackCtrl_WithLimit(
                    pos_rot, 0f,
                    0f, k_stiff_radial_throt, B_DAMP_ROT_TRACKING,
                    K_STIFF_ROT_BASE);

                // Command offset forces for ASSISTANCE - CRITICAL:
                SetOffsetForces(0f, torque_assist);

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
                    pos_rot, 0f, 
                    0f, K_STIFF_RADIAL_THROT_MANUAL, 0f,
                    K_STIFF_ROT_BASE);
                break;

            default: // no command
                break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Control commands using Set Target - functions:
    ////////////////////////////////////////////////////////////////////////////

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

        if (ExerciseActive)
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

        k_stiff_rot_equiv = k_stiff_rot_ref + k_stiff_rot_lim + k_stiff_rot_base;

        pos_eq_rot_equiv = (k_stiff_rot_ref*pos_rot_eq_ref + k_stiff_rot_lim*pos_rot_eq_lim)
             / k_stiff_rot_equiv;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        if (ExerciseActive)
            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_THROT_ZERO_OFFS, pos_eq_rot_equiv,
                k_stiff_radial_throt, k_stiff_rot_equiv,
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
                // Added this disengage command because the one in StartExercise() apparently has no effect (20.08.2025):
                SetBrakes(DISENGAGE_BRAKE, DISENGAGE_BRAKE);

                // Display section:
                ExternalConsoleLogger.Log("        StartExercise(): SetBrakes(): before MotionRoutineRHBSimple - cmd DISENGAGE \n");

                // Move end effector to radial baseline position:
                bool success_all = MotionRoutineRadialRHBBaseline();

                // Display section:
                ExternalConsoleLogger.Log("        --------------------------------------------------------------------");
                ExternalConsoleLogger.Log("        MotionRoutineRadialRHBBaseline() EXECUTED, success all [" + success_all + "] \n");
            });
        }

        // Stop exercise:
        else
        {
            StopExercise();
        }
    }

    private void StartExercise(bool unlockRadial, bool unlockRotational, UnityAction onComplete = null)
    {
        if (isExerciseStarted)
        {             
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("....................................................................");
            ExternalConsoleLogger.Log("StartExercise(): isExerciseStarted  - return \n");

            onComplete?.Invoke();

            if (isCalibrated)
                OnExerciseStart?.Invoke();

            return;
        }

        bool startExerciseSucess = false;
        const int N_ATTEMPTS_MAX = 50;

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

            Thread.Sleep(10);
        }
        while (++i <= N_ATTEMPTS_MAX && !startExerciseSucess);
           
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

        /////////////////////////////////////////////////////////
        // Stop timer:
        /////////////////////////////////////////////////////////
        
        timerLocked = true;
        timerActivePrev = timerActive;
        timerActive = false;
        System.Threading.Thread.Sleep(DT_STEP_APP_MSEC);
        timerLocked = false;

        /////////////////////////////////////////////////////////
        // Set default feedback gains:
        /////////////////////////////////////////////////////////
        
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

        /////////////////////////////////////////////////////////
        // Move RHB end effector to 'home' position (with minimum radial position);
        /////////////////////////////////////////////////////////
        
        SetBrakes(DISENGAGE_BRAKE, DISENGAGE_BRAKE);
        ExternalConsoleLogger.Log("        StopExercise(): SetBrakes(): before MotionRoutineRHBSimple - cmd DISENGAGE \n");

        bool success_all = MotionRoutineRHBSimple(POS_RADIAL_MIN_OFFS);

        ExternalConsoleLogger.Log("        --------------------------------------------------------------------");
        ExternalConsoleLogger.Log("        MotionRoutineRHBSimple() radial target [" + POS_RADIAL_MIN_OFFS + "] EXECUTED, success all [" + success_all + "] \n");
    
        SetBrakes(ENGAGE_BRAKE, ENGAGE_BRAKE);
        ExternalConsoleLogger.Log("        StopExercise(): SetBrakes(): after MotionRoutineRHBSimple - cmd ENGAGE \n");

        isExerciseStarted = false;
        isExerciseStopping = false;

        /////////////////////////////////////////////////////////
        // Set DataManager 'race started' flag (27.08.2025):
        /////////////////////////////////////////////////////////
        
        DataManager.instance.isRaceStarted = false;

        OnExerciseStop?.Invoke();
        onComplete?.Invoke();

        /////////////////////////////////////////////////////////
        // Go back to menu and restart game - TODO: implement proper game restart for Care Platform:
        /////////////////////////////////////////////////////////

        isExerciseStarted = false;
        isCalibrated = false;

        STATE_PREGAME = ST_SELECT_BIKE_TYPE;

        SceneManager.LoadScene(0);
        Time.timeScale = 1f;
        DOTween.PlayAll();

        OnConnect_PreUnityGame();

        /////////////////////////////////////////////////////////
        // Replaced by MotionRoutineRHBSimple() (22.08.2025):
        /////////////////////////////////////////////////////////
        
        /*
        motionRoutineRotational = StartCoroutine(MotionRoutineRotationalRHB(0f, () =>
        {
            motionRoutineRadial = StartCoroutine(MotionRoutineRadialRHB(POS_RADIAL_MIN, () =>
            {
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

        radialValue = Mathf.Clamp(radialValue, POS_RADIAL_MIN_OFFS, POS_RADIAL_MAX);
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
            POS_RADIAL_MIN_OFFS, 0f,
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
                POS_RADIAL_THROT_ZERO_OFFS, POS_ROT_BASE,
                K_STIFF_RADIAL_THROT_MANUAL, K_STIFF_ROT_BASE,
                B_DAMP_RADIAL_BASE, B_DAMP_ROT_BASE,
                switch_var, 1f);

            if (success_all && !success_step)
                success_all = success_step;

            Thread.Sleep(DT_MOTION_BASE_MSEC / N_STEPS_MOTION_BASE);

            switch_var += 1f / N_STEPS_MOTION_BASE;
        }

        return success_all;
    }
}