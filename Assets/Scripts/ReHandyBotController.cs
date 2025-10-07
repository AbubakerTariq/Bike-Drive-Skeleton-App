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
    // CARE_PLATFORM controlled parameters - MANUAL throttle parameters:
    //////////////////////////////////////////////////////////////////////////// 

    // Handles opening distance (meters) for zero throttle input:
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient hand opening data; keep default for first CARE_PLATFORM release)
    public float POS_RADIAL_THROT_ZERO = 0.029f;

    // Throttle stiffness for MANUAL throttle mode:
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient stiffness calibration data; keep default for first CARE_PLATFORM release)
    private float K_STIFF_RADIAL_THROT_MANUAL;  

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

    private float FORCE_GAIN_RADIAL =  9.0f;
    private float FORCE_GAIN_ROT    =  6.0f; // 14.0f; // Reduced gain for greater stability with ASSISTED and MANUAL control

    private float K_STIFF_RADIAL_WALL = 2500f; // use with zero feedback gain

    // Extra damping to prevent limit cycles when handles contact physical limit
    // Normally should rely on embedded HL_SetTarget stability
    // TODO: check why limit cycle suppression doesn't act in firmware (29.09.2025):
    private float B_DAMP_RADIAL_WALL  = 0f; 

    private float K_STIFF_ROT_WALL = 1.2f; // use with zero feedback gain
    private float B_DAMP_ROT_WALL  = 0f; // rely on embedded HL_SetTarget stability

    private float POS_RADIAL_MIN = 0.0145f;
    private float POS_RADIAL_MAX = 0.06f;

    // Offset of the MINIMUM RADIAL position to account for initial calibration errors (PD discussion on 20.09.2025):
    private float POS_RADIAL_MIN_OFFS;

    private float POS_ROT_MIN = -Mathf.PI / 2f;
    private float POS_ROT_MAX =  Mathf.PI / 2f;

    // Throttle - additional haptics settings:
    private float K_STIFF_RADIAL_THROT_AUTO = 5000f; // makes handles essentially rigid
    private float B_DAMP_RADIAL_BASE = 0f; // rely on embedded HL_SetTarget stability 

    // Steering - BASELINE haptics settings:
    private const float POS_ROT_BASE = 0f;

    private float K_STIFF_ROT_BASE = 0.05f; // 0.1f; // 
    private float B_DAMP_ROT_BASE  = 0f; // rely on embedded HL_SetTarget stability

    // Stiffness for TRACKING control
    // NOTE: check stabilizing damping for K_STIFF_ROT_TRACKING + K_STIFF_ROT_BASE
    // (use RHB ctrl params - stability v5b game settings 4-axis)
    private float K_STIFF_ROT_TRACKING = 2.2f;
    private float B_DAMP_ROT_TRACKING = 0.05f; // 0.045f; // 0.040f; // 

    // Stiffness for ASSISTIVE control - fraction of TRACKING stiffness:
    public float FRAC_ASSIST_STIFF = 0.5f; // 0.45f; // 0.35f; // 

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - STEERING assistance:
    //////////////////////////////////////////////////////////////////////////// 

    // Maximum STEERING ASSIST torque - CRITICAL:
    const float TORQUE_ASSIST_STEER_MAX = 0.18f; // 0.2f; // 0.1f;

    // ASSIST FACTOR (between 0 and 1.0) - modified computation after user feedbacks (29.09.2025)
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM game level)
    public float FACT_ASSIST_STEER = 0f;

    private const int GAME_LEVEL_MID = 5;
    private const float FACT_ASSIST_MID = 0.3f;

    // Scaling factor for Patient's rotational inputs
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM patient ROM data)
    public float FRAC_POS_ROT_INPUT_PATIENT = 0.4f;

    ////////////////////////////////////////////////////////////////////////////
    // CARE_PLATFORM controlled parameters - THROTTLE assistance:
    //////////////////////////////////////////////////////////////////////////// 

    public const int THROTTLE_MODE_MANUAL = 0;
    public const int THROTTLE_MODE_AUTO   = 1;

    // Throttle mode (0: MANUAL, 1: AUTO, default is AUTO)
    // To be set by UNITY_GAME or CARE_PLATFORM (compute using CARE_PLATFORM throttle setting)
    public float FACT_ASSIST_THROTTLE = (float)THROTTLE_MODE_AUTO;

    ////////////////////////////////////////////////////////////////////////////
    // RHB motion limits: stiffness & angle
    ////////////////////////////////////////////////////////////////////////////   

    private float K_STIFF_ROT_LIM  = 0.6f;

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

    public int step_count;

    //////////////////////////////////////////////////////////////////////////// 
    // NEW: GAME LEVELS (29.09.2025)
    //////////////////////////////////////////////////////////////////////////// 

    public const int N_GAME_LEVELS = 10;

    public int game_level_curr = 1; // default value
    public int game_level_next;

    ////////////////////////////////////////////////////////////////////////////
    // NEW: RACE DIRECTION (29.09.2025)
    //////////////////////////////////////////////////////////////////////////// 

    public const int DIR_CW  = +1; // clockwise direction
    public const int DIR_CCW = -1; // counterclockwise direction 

    public int RACE_DIRECTION;

    ////////////////////////////////////////////////////////////////////////////
    // NEW: PERFORMANCE variables (29.09.2025)
    ////////////////////////////////////////////////////////////////////////////

    // UNDERSTEER parameters:
    const float FRAC_UNDERSTEER_LEVEL_UP_MAX   = 0.33f; // if understeer fraction is less than this, go up one level
    const float FRAC_UNDERSTEER_LEVEL_DOWN_MIN = 0.67f; // if understeer fraction is more than this, go down one level

    public float frac_understeer = -1f;

    // DISTANCE TRAVELED parameters:
    // "Legitimate" race for performace purposes
    // Traveled distance must be greater than this fraction of the track length 
    const float FRAC_LENGTH_TRACK_LEGIT_RACE = 0.9f;

    public float frac_dist_traveled = 0f;

    // FALL parameters:
    // Number of falls allowed without triggering GAME LEVEL reduction:   
    const int N_FALLS_LIM = 1;

    // Game level change value (-1, 0, +1): 
    private int game_level_change = 0;

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
            ////////////////////////////////////////////////////////////////////////////
            // Recommend GAME LEVEL change based on last exercise PERFORMANCE:
            // UNDERSTEER fraction / bike falling / distance traveled:
            ////////////////////////////////////////////////////////////////////////////
            
            int game_level_next;

            if (frac_understeer >= 0) // frac_understeer < 0 means the game hasn't started - no PERFORMANCE metric computed yet
            {
                game_level_next = PerformGameLevelChange(
                    game_level_curr, N_GAME_LEVELS, ref game_level_change,
                    frac_understeer,
                    MotorbikeController.instance.step_count_fall,
                    MotorbikeController.instance.dist_traveled, Track.instance.GetTrackLength());

                // TODO: perform this assignment when using automated GAME LEVEL update:
                // GAME_LEVEL_CURR = game_level_next; 
            }

            ////////////////////////////////////////////////////////////////////////////
            // Display section - GAME LEVEL selection message
            ////////////////////////////////////////////////////////////////////////////

            string str_performance;

            if (frac_understeer >= 0) 
                str_performance =
                    "Previous exercise PERFORMANCE: \n\n" +

                    "UNDERSTEER fraction   = [" + String.Format("{0:#0.0}", 100 * frac_understeer)                + " %] \n" +
                    "   max for level UP   = [" + String.Format("{0:#0.0}", 100 * FRAC_UNDERSTEER_LEVEL_UP_MAX)   + " %] \n" +
                    "   min for level DOWN = [" + String.Format("{0:#0.0}", 100 * FRAC_UNDERSTEER_LEVEL_DOWN_MIN) + " %] \n" +

                    "# falls = [" + MotorbikeController.instance.step_count_fall + "] (limit = " + N_FALLS_LIM + ")\n\n" +

                    "Dist traveled = [" + 
                        String.Format("{0:#0}",   MotorbikeController.instance.dist_traveled) + " m] = " +
                        String.Format("{0:#0.0}", 100f * MotorbikeController.instance.dist_traveled / Track.instance.GetTrackLength()) + 
                        " % track length (min = " + 100f*FRAC_LENGTH_TRACK_LEGIT_RACE + "%) \n\n" +

                    "Current GAME LEVEL       = [" + game_level_curr + "] \n" +
                    "Recommended LEVEL CHANGE = [" + game_level_change + "] \n\n";
            else
                str_performance = "\n\n";

            loaderText.text =
                str_performance +
                "\n\n" +
                "Select GAME LEVEL, 1 to 10\n" +
                "(enter [0] for 10) \n";
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
            string str_game_level;
            string str_fact_assist;
            string str_race_direction;
            string str_game_settings;

            if (USE_BEGINNER_BIKE_CONSTR)
                str_bike_type = "Bike type: BEGINNER";
            else
                str_bike_type = "Bike type: PRO";

            if (CASE_CTRL_MODE == CTRL_ASSISTED)
            {
                str_game_level = "GAME LEVEL = " + game_level_curr;

                str_fact_assist =
                    "Assist factor STEERING = " + FACT_ASSIST_STEER + "\n" +
                    "Assist factor THROTTLE = " + FACT_ASSIST_THROTTLE;

                str_race_direction = "RACE_DIRECTION = [" + RACE_DIRECTION + "]";

                str_game_settings =
                    str_game_level + "\n" +
                    str_fact_assist + "\n" +
                    str_race_direction;
            }
            else
                str_game_settings = " ";

            loaderText.text =
                str_bike_type +"\n\n" +
                "CONTROL MODE [" + CASE_CTRL_MODE + "]\n\n" +
                str_game_settings + "\n\n" +
                "Align grippers horizontally and close the grippers \n\n" +
                "Press Y to CALIBRATE";
        }
    }

    private void InitUnityGame_StartExercise(
        ref bool use_beginner_bike_constr,
        ref int case_ctrl_mode,
        ref int game_level,
        ref float fact_assist_steer,
        ref float fact_assist_throttle,
        ref float frac_pos_rot_input_patient,
        ref float pos_radial_throt_zero,
        ref float k_stiff_radial_throt_manual,
        ref float speed_auto_throttle_max_kph,
        ref int race_direction,
        ref bool upright_constr_on)
    {
        ////////////////////////////////////////////////////////////////////////////
        // Fixed settings for UNITY_GAME:
        ////////////////////////////////////////////////////////////////////////////

        frac_pos_rot_input_patient = 0.4f;
        pos_radial_throt_zero = 0.029f;
        k_stiff_radial_throt_manual = 2500f;
        speed_auto_throttle_max_kph = 150f; // 125f;

        ////////////////////////////////////////////////////////////////////////////
        // Selectable settings for UNITY_GAME:
        ////////////////////////////////////////////////////////////////////////////

        if (STATE_PREGAME != ST_CALIBRATE) 
        { 
            SelectGameSettings_PreUnityGame(
                ref use_beginner_bike_constr,
                ref case_ctrl_mode,
                ref game_level,
                ref fact_assist_throttle,
                ref race_direction,
                OnSelectGameSettings_PreUnityGame);

            // Convert game level to assistance factor (fraction)
            // Modified computation after user feedbacks (29.09.2025)
            fact_assist_steer = FactorAssistCalc(game_level);
        }

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
        ref int game_level,
        ref float fact_assist_throttle,
        ref int race_direction,
        UnityAction onComplete = null)
    {
        // TODO: implement selection of race direction:
        race_direction = DIR_CW;

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
        // Select GAME LEVEL manually
        //
        // TODO: this should be replaced by AUTOMATED GAME LEVEL change based on PERFORMANCE
        // See OnSelectGameSettings_PreUnityGame() / if (STATE_PREGAME == ST_SET_FACT_ASSIST_STEER)
        ////////////////////////////////////////////////

        else if (STATE_PREGAME == ST_SET_FACT_ASSIST_STEER)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1))
                game_level = 1;
            else if (Input.GetKeyDown(KeyCode.Alpha2))
                game_level = 2;
            else if (Input.GetKeyDown(KeyCode.Alpha3))
                game_level = 3;
            else if (Input.GetKeyDown(KeyCode.Alpha4))
                game_level = 4;
            else if (Input.GetKeyDown(KeyCode.Alpha5))
                game_level = 5;
            else if (Input.GetKeyDown(KeyCode.Alpha6))
                game_level = 6;
            else if (Input.GetKeyDown(KeyCode.Alpha7))
                game_level = 7;
            else if (Input.GetKeyDown(KeyCode.Alpha8))
                game_level = 8;
            else if (Input.GetKeyDown(KeyCode.Alpha9))
                game_level = 9;
            else if (Input.GetKeyDown(KeyCode.Alpha0))
                game_level = 10;
            else
                return;

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
        // NOTE: both functions must set the "bike upright" constraint (UPRIGHT_CONSTR_ON) to true before the exercise is launched:
        ////////////////////////////////////////////////////////////////////////////

        InitUnityGame_StartExercise(
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

        // Write game PARAMETERS to Data Manager structure (TODO: revise implementation in Care Platform version):
        if (!ExerciseActive)
            CopyParamsToStruct(ref DataManager.instance.params_values);

        // Offset the RADIAL reference positions to account for initial calibration errors - CRITICAL (20.09.2025):
        POS_RADIAL_THROT_ZERO_OFFS = POS_RADIAL_THROT_ZERO +   OFFS_POS_RADIAL_CALIB;
        POS_RADIAL_MIN_OFFS        = POS_RADIAL_MIN        + 2*OFFS_POS_RADIAL_CALIB;

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

        float pos_radial = distal_data.PositionR;

        float pos_rot    = distal_data.PositionP;
        float dt_pos_rot = distal_data.VelocityP;

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
                    pos_rot, dt_pos_rot, // was dt_pos_rot_kal (24.09.2025)
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
        float pos_rot_est, float dt_pos_rot_est, 
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
                
                ///////////////////////////////////////////////////////////////////////////
                // Compute assistive torque:
                ///////////////////////////////////////////////////////////////////////////
                
                // Formula updated after updating offset force cmd in firmware (24.09.2025):
                float TORQUE_ASSIST_LIM  = FACT_ASSIST_STEER * TORQUE_ASSIST_STEER_MAX;

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

        pos_eq_rot_equiv = 
            (k_stiff_rot_ref*pos_rot_eq_ref + k_stiff_rot_lim*pos_rot_eq_lim)
             / k_stiff_rot_equiv;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        // TODO: consider change to plain distalRobot.SetTarget to reduce overhead (possible risk is timeouts):
        if (ExerciseActive)
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
    
    int PerformGameLevelChange(
        int game_level_curr, int num_game_levels, ref int level_change,
        float frac_underst,
        int step_count_fall,
        float dist_traveled, float length_track)
    {
        if (dist_traveled < FRAC_LENGTH_TRACK_LEGIT_RACE*length_track)
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

    float FactorAssistCalc(int game_level)
    {
        float fact_assist_steer;

        // Slopes for piecewise computation:
        float m1 = (1f - FACT_ASSIST_MID) / GAME_LEVEL_MID;
        float m2 = FACT_ASSIST_MID / GAME_LEVEL_MID;

        if (game_level <= GAME_LEVEL_MID)
            fact_assist_steer = 1f - m1*(float)game_level;
        else
            fact_assist_steer = m2*(N_GAME_LEVELS - (float)game_level);

        return fact_assist_steer;
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

        //////////////////////////////////////////////////////////////////
        // Send Start Exercise command to firmware:
        //////////////////////////////////////////////////////////////////

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

        //////////////////////////////////////////////////////////////////
        // Start Exercise procedures:
        //////////////////////////////////////////////////////////////////

        if (startExerciseSucess) {
            isExerciseStarted = true;

            //////////////////////////////////////////////////////////////////
            // Start timer:
            //////////////////////////////////////////////////////////////////
            
            timerLocked = true;
            timerActivePrev = timerActive;
            timerActive = true;
            System.Threading.Thread.Sleep(DT_STEP_APP_MSEC);
            timerLocked = false;

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
            MotorbikeController.instance.step_count_fall  = 0;
            MotorbikeController.instance.bike_fallen_prev = false;

            // Distance traveled during exercise:
            MotorbikeController.instance.dist_traveled = 0f;

            //////////////////////////////////////////////////////////////////
            // Set force feedback gains:
            //////////////////////////////////////////////////////////////////

            SetForceFeebackGainCases(CASE_CTRL_MODE);

            //////////////////////////////////////////////////////////////////
            // Initiate recording on new data file:
            //////////////////////////////////////////////////////////////////
            
            DataManager.instance.SetupRecordingEvents();

            //////////////////////////////////////////////////////////////////
            // Display section:
            //////////////////////////////////////////////////////////////////

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

        //////////////////////////////////////////////////////////////////
        // PERFORMANCE metrics: compute UNDERSTEER fraction
        //////////////////////////////////////////////////////////////////

        frac_understeer = (float)MotorbikeController.instance.step_count_understeer / step_count;

        /////////////////////////////////////////////////////////
        // Set default feedback gains:
        /////////////////////////////////////////////////////////

        SetGain(0f, 0f); // was SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT) (29.09.2025)

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
        // Go back to menu and restart game
        // NOTE: proper game restart rquired for Care Platform
        /////////////////////////////////////////////////////////

        isExerciseStarted = false;
        isCalibrated = false;

        STATE_PREGAME = ST_SELECT_BIKE_TYPE;

        SceneManager.LoadScene(0);
        Time.timeScale = 1f;
        DOTween.PlayAll();

        OnConnect_PreUnityGame();
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

    // Write game PARAMETERS to Data Manager structure (TODO: revise implementation in Care Platform version):
    public void CopyParamsToStruct(ref DataManager.ParamsBikeGame params_values_this) { 
        params_values_this.DT_STEP_APP_MSEC = DT_STEP_APP_MSEC;
        params_values_this.USE_BEGINNER_BIKE_CONSTR = USE_BEGINNER_BIKE_CONSTR;
        params_values_this.CASE_CTRL_MODE = CASE_CTRL_MODE;
        params_values_this.POS_RADIAL_THROT_ZERO = POS_RADIAL_THROT_ZERO;
        params_values_this.K_STIFF_RADIAL_THROT_MANUAL = K_STIFF_RADIAL_THROT_MANUAL;
        params_values_this.SPEED_AUTO_THROTTLE_MAX_KPH = SPEED_AUTO_THROTTLE_MAX_KPH;
        params_values_this.FORCE_GAIN_RADIAL = FORCE_GAIN_RADIAL;
        params_values_this.FORCE_GAIN_ROT = FORCE_GAIN_ROT;
        params_values_this.K_STIFF_RADIAL_THROT_AUTO = K_STIFF_RADIAL_THROT_AUTO;
        params_values_this.K_STIFF_ROT_BASE = K_STIFF_ROT_BASE;
        params_values_this.K_STIFF_ROT_TRACKING = K_STIFF_ROT_TRACKING;
        params_values_this.B_DAMP_ROT_TRACKING = B_DAMP_ROT_TRACKING;
        params_values_this.FRAC_ASSIST_STIFF = FRAC_ASSIST_STIFF;
        params_values_this.TORQUE_ASSIST_STEER_MAX = TORQUE_ASSIST_STEER_MAX;
        params_values_this.FACT_ASSIST_STEER = FACT_ASSIST_STEER;
        params_values_this.GAME_LEVEL_MID = GAME_LEVEL_MID;
        params_values_this.FACT_ASSIST_MID = FACT_ASSIST_MID;
        params_values_this.FRAC_POS_ROT_INPUT_PATIENT = FRAC_POS_ROT_INPUT_PATIENT;
        params_values_this.FACT_ASSIST_THROTTLE = FACT_ASSIST_THROTTLE;
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