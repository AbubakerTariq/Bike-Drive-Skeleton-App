using Articares.Distal;
using DG.Tweening;
// using UnityEngine.UI;
// using UnityEngine.Video;
using System;
using System.Collections;
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

    // Set Target command time step (TODO: keep or discard - together with StartSetTargetEvents() ):
    public const int DT_STEP_SET_TARG_MSEC = 25;

    ////////////////////////////////////////////////////////////////////////////
    // SetExercise() parameters (CRITICAL):
    //////////////////////////////////////////////////////////////////////////// 

    public const int NUM_TARGETS = 1;

    float OFFS_FORCE_RADIAL_INIT = 0f;
    float OFFS_TORQUE_ROT_INIT = 0f;

    private bool SAFETY_SET_TARG = false;
    private bool STABILITY_SET_TARG = true;

    ////////////////////////////////////////////////////////////////////////////
    // Target indices:
    ////////////////////////////////////////////////////////////////////////////

    private byte IDX_TARG_BASE = 1;
    private byte IDX_TARG_LIM = 2;

    ////////////////////////////////////////////////////////////////////////////
    // Object instances:
    ////////////////////////////////////////////////////////////////////////////

    public static ReHandyBotController instance;
    private DistalComm distalRobot = new(); // Distal Control Library object

    // Track object:
    Track track;

    // Bike object:
    MotorbikeController bike;

    ////////////////////////////////////////////////////////////////////////////
    // Configuration values:
    ////////////////////////////////////////////////////////////////////////////

    private float FORCE_GAIN_RADIAL = 9f;
    private float FORCE_GAIN_ROT = 14f;

    private float K_STIFF_RADIAL_WALL = 2500f; // use with zero feedback gain
    private float B_DAMP_RADIAL_WALL = 40f;

    private float K_STIFF_ROT_WALL = 1.2f; // use with zero feedback gain
    private float B_DAMP_ROT_WALL = 0.092f;

    private float POS_RADIAL_MIN = 0.0145f;
    private float POS_RADIAL_MAX = 0.06f;

    private float POS_ROT_MIN = -Mathf.PI / 2f;
    private float POS_ROT_MAX = Mathf.PI / 2f;

    ////////////////////////////////////////////////////////////////////////////
    // RHB control settings - CRITICAL
    // NOTE: use [RHB ctrl params - stability v5b game settings 4-axis.xlsx] to calculate damping as a function of stiffness
    ////////////////////////////////////////////////////////////////////////////

    // Throttle - BASELINE haptics settings:
    [HideInInspector] public float POS_RADIAL_BASE_THROT = 0.029f;
    private float K_STIFF_RADIAL_BASE_THROT = 2500f;
    private float B_DAMP_RADIAL_BASE_THROT = 21.0f;

    // Steering - BASELINE haptics settings:
    [HideInInspector] public float POS_ROT_BASE_STEER = 0f;
    private float K_STIFF_ROT_BASE_STEER = 0.1f; // 0.15f; 
    private float B_DAMP_ROT_BASE_STEER = 0.015f; // 0.0185f;

    ////////////////////////////////////////////////////////////////////////////
    // Impedance for RHB motion limits:
    ////////////////////////////////////////////////////////////////////////////   

    private float K_STIFF_ROT_LIMIT = 0.3f;
    private float B_DAMP_ROT_LIMIT = 0f; // is the minimum stability value enough?

    public float ANGLE_ROT_LIM_DEG = 45f;

    ////////////////////////////////////////////////////////////////////////////
    // Data structures from bike and track objects:
    ////////////////////////////////////////////////////////////////////////////

    static Vector3 NULL_VECTOR3 = new Vector3(0f, 0f, 0f);
    const float NULL_VALUE = 0f;
 
    public Vector3 pos_bike = NULL_VECTOR3;
    public Vector3 vect_dir_bike = NULL_VECTOR3;
    public Vector3 dt_pos_bike = NULL_VECTOR3;

    public Vector3 pos_ctrline_near = NULL_VECTOR3;
    public Vector3 vect_ctrline_tang = NULL_VECTOR3;
    public float curv_ctrline_near = NULL_VALUE;
    public float ang_ctrline_tang = NULL_VALUE;
    public float dist_ctrline_near = NULL_VALUE; 

    // public struct BikeData
   
    // public struct TrackData;

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
    public DistalComm.ExerciseData DistalData => distalRobot.DistalData;
    public bool ExerciseActive => isExerciseStarted;

    ////////////////////////////////////////////////////////////////////////////
    // Exercise-related variables:
    ////////////////////////////////////////////////////////////////////////////

    private bool isSystemStarted = false;
    private bool isExerciseStarted = false;
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

    private Coroutine motionRoutineRadial;
    private Coroutine motionRoutineRotational;

    private bool isMoving = false;
    private bool isRotating = false;

    private float minPinch;
    // private float maxPinch;

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
    // Thread and timer dor SetTarget process:
    ////////////////////////////////////////////////////////////////////////////

    private System.Timers.Timer timerSetTarget;
    private Thread threadTimerSetTarget;

    ////////////////////////////////////////////////////////////////////////////
    // Data display:
    ////////////////////////////////////////////////////////////////////////////

    private int DT_DISP_DATA_MSEC = 2000;
    private bool DISP_TIMER_ACTIVITY_ON = true;

    ////////////////////////////////////////////////////////////////////////////
    // Methods section:
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
        // Stop Set target events:
        StopSetTargetEvents();

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

    private void Update()
    {
        ////////////////////////////////////////////////////////////////////////////
        // State check:
        ////////////////////////////////////////////////////////////////////////////

        // If robot is calibrated and user presses Enter, exercise state will be toggled
        // The exercise will start if it hasn't started already.  
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
        // Command steer angle limits:
        ////////////////////////////////////////////////////////////////////////////

        CmdSetTargetSteerLimit();

        ////////////////////////////////////////////////////////////////////////////
        // Extract data from bike and track objects:
        ////////////////////////////////////////////////////////////////////////////

        if (ExerciseActive && MotorbikeController.Instance != null && Track.Instance != null)
        {
            bike = MotorbikeController.Instance;

            pos_bike = bike.GetBikePosition();
            vect_dir_bike = bike.GetBikeDirectionVector();
            dt_pos_bike = bike.GetBikeVelocityVector();

            track = Track.Instance;

            pos_ctrline_near = track.GetClosestPointOnCenterLine(pos_bike);
            vect_ctrline_tang = track.GetTangentAtPosition(pos_bike);

            curv_ctrline_near = track.GetCurvatureAtPosition(pos_bike);
            ang_ctrline_tang = (float)Math.PI / 180f * track.GetTangentAngleAtPosition(pos_bike);
            dist_ctrline_near = track.GetDistanceAtPosition(pos_bike);  

            // track.GetTrackLength();
        }
    
        ////////////////////////////////////////////////////////////////////////////
        // Display section:
        ////////////////////////////////////////////////////////////////////////////

        // Time elapsed display:
        string timeElapsedText = String.Format("{0:#00}", timeElapsedSpan.Minutes) + ":" + String.Format("{0:#00}", timeElapsedSpan.Seconds);

        if (step_count % (DT_DISP_DATA_MSEC / DT_STEP_APP_MSEC) == 0 && DISP_TIMER_ACTIVITY_ON)
        {
            ExternalConsoleLogger.Log("Update(" + step_count + ") t [" + String.Format("{0:#0.000}", timeElapsedValue) + "]:");
            ExternalConsoleLogger.Log("   pos bike " + pos_bike      );
            ExternalConsoleLogger.Log("   dir bike " + vect_dir_bike );
            ExternalConsoleLogger.Log("   vel bike " + dt_pos_bike   );
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("   pos near  " + pos_ctrline_near  );
            ExternalConsoleLogger.Log("   vect tang " + vect_ctrline_tang );
            ExternalConsoleLogger.Log("   curvature [" + String.Format("{0:#0.000}", curv_ctrline_near) + "]");
            ExternalConsoleLogger.Log("   ang tang  [" + String.Format("{0:#0.00}", ang_ctrline_tang)   + "]");
            ExternalConsoleLogger.Log("   d ctrline [" + String.Format("{0:#0.00}", dist_ctrline_near)   + "]");
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

    #region Exercise tasks
    private void CmdSetTargetSteerLimit()
    {
        ////////////////////////////////////////////////////////////////////////////
        // RHB coordinates:
        //////////////////////////////////////////////////////////////////////////// 

        float pos_radial = ReHandyBotController.instance.DistalData.PositionR;
        float pos_phi = ReHandyBotController.instance.DistalData.PositionP;

        ////////////////////////////////////////////////////////////////////////////
        // Impedance parameters for rotation angle limit:
        ////////////////////////////////////////////////////////////////////////////

        // RADIAL parameters:
        /*
        float pos_radial_lim = POS_RADIAL_MIN;
        float k_stiff_radial_lim = 0f;
        float b_damp_radial_lim = 0f;
        */

        float gain_radial = 1.0f;

        // ROTATIONAL parameters:
        float ANGLE_ROT_LIM = ANGLE_ROT_LIM_DEG * (float)Math.PI / 180f;

        float pos_rot_lim;
        float k_rot_lim;
        float b_rot_lim;

        float gain_rot = 1.0f;

        ////////////////////////////////////////////////////////////////////////////
        // Compute impedance parameters:
        ////////////////////////////////////////////////////////////////////////////

        // If rotation limit exceeded, apply nonzero values to rotational siffness and damping:
        if ((pos_phi - ANGLE_ROT_LIM > 0f) || (pos_phi + ANGLE_ROT_LIM < 0f))
        {
            k_rot_lim = K_STIFF_ROT_LIMIT;
            b_rot_lim = B_DAMP_ROT_LIMIT;
        }
        else
        {
            k_rot_lim = 0f;
            b_rot_lim = 0f;
        }

        if (pos_phi - ANGLE_ROT_LIM > 0f)
            pos_rot_lim = ANGLE_ROT_LIM;
        else if (pos_phi + ANGLE_ROT_LIM < 0f)
            pos_rot_lim = -ANGLE_ROT_LIM;
        else
            pos_rot_lim = 0f;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        bool success_set_target;

        if (ExerciseActive)
        {
            // success_set_target = HL_SetTargetEmpty();

            success_set_target = distalRobot.HL_SetTarget(IDX_TARG_BASE,
                POS_RADIAL_BASE_THROT, pos_rot_lim,
                K_STIFF_RADIAL_BASE_THROT, k_rot_lim,
                B_DAMP_RADIAL_BASE_THROT, b_rot_lim,
                gain_radial, gain_rot);
        }
        else
            success_set_target = false;

        ////////////////////////////////////////////////////////////////////////////
        // Display section: 
        ////////////////////////////////////////////////////////////////////////////

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
                "stiff: RAD [" + String.Format("{0:#0.0}", K_STIFF_RADIAL_BASE_THROT) + "]  ROT [" + String.Format("{0:#0.000}", k_rot_lim) + "] \n" +
                "damp:  RAD [" + String.Format("{0:#0.0}", B_DAMP_RADIAL_BASE_THROT) + "]  ROT [" + String.Format("{0:#0.000}", b_rot_lim) + "] \n");
        }
    }
    #endregion

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
        SetBrakes(true, true);

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
            distalRobot.SetSafety(SAFETY_SET_TARG);
            onComplete?.Invoke();
            return;
        }

        for (int i = 0; i < MaxAttempts; i++)
        {
            bool success = distalRobot.StartSystem();

            if (success)
            {
                distalRobot.SetSafety(SAFETY_SET_TARG);
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
        StartExercise(false, false, () =>
        {
            DOVirtual.DelayedCall(0.1f, () =>
            {
                loader.SetActive(false);
                minPinch = DistalData.PositionR;
                minPinch = Math.Clamp(minPinch, POS_RADIAL_MIN, POS_RADIAL_MAX);

                for (int i = 0; i < MaxAttempts; i++)
                {
                    bool success = distalRobot.StopExercise();

                    if (success)
                    {
                        SetBrakes(false, false);
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
        if (isExerciseStarted)
            StopExercise();
        else
        {
            // TODO: find out why calling StartExercise() a second time fails:
            StartExercise(true, true, () =>
            {
                MotionRoutineRadialRHBBaseline();
            });
        }
    }

    private void StartExercise(bool unlockRadial, bool unlockRotational, UnityAction onComplete = null)
    {
        // bool timer_started = false;
        bool success_set_targ_empty;

        if (isExerciseStarted)
        {
            SetBrakes(unlockRadial, unlockRotational);

            success_set_targ_empty = HL_SetTargetEmpty();

            onComplete?.Invoke();

            if (isCalibrated)
                OnExerciseStart?.Invoke();

            return;
        }

        for (int i = 0; i < MaxAttempts; i++)
        {
            distalRobot.HL_StartExercise(
                NUM_TARGETS, unlockRadial, unlockRotational,
                OFFS_FORCE_RADIAL_INIT, OFFS_TORQUE_ROT_INIT,
                out bool startExerciseResponse, out bool setGainResponse,
                FORCE_GAIN_RADIAL, FORCE_GAIN_ROT, STABILITY_SET_TARG);

            if (!startExerciseResponse)
            {
                if (!distalRobot.LastErrorMessage.Contains("Timeout while waiting for StartResumeExercise response"))
                    continue;

                // Removed HL_SetTargetEmpty() loop routine (15.08.2025):
                /*
                int fail_set_targ_count = 0;
                while (fail_set_targ_count < MaxAttempts)
                {
                    if (!HL_SetTargetEmpty())
                        fail_set_targ_count++;
                    else
                    {
                        fail_set_targ_count = 0;
                        break;
                    }
                }

                if (fail_set_targ_count >= MaxAttempts)
                    continue;
                */
            }

            isExerciseStarted = true;

            // Start timer:
            timerLocked = true;
            timerActivePrev = timerActive;
            timerActive = true;
            System.Threading.Thread.Sleep(DT_STEP_APP_MSEC);
            timerLocked = false;

            // Display section:
            ExternalConsoleLogger.Log(" ");
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("StartExercise(): timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");

            if (isCalibrated)
                OnExerciseStart?.Invoke();

            onComplete?.Invoke();

            if (setGainResponse)
                break;

            SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
            break;
        }
    }

    private void StopExercise(UnityAction onComplete = null)
    {
        if (!isExerciseStarted)
        {
            SetBrakes(false, false);
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

        // Display section:
        ExternalConsoleLogger.Log(" ");
        ExternalConsoleLogger.Log("____________________________________________________________________");
        ExternalConsoleLogger.Log("StopExercise(): timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");

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

        SetBrakes(true, true);

        // Move RHB end effector to 'home' position (with minimum radial position);
        MotionRoutineRHBSimple(POS_RADIAL_MIN);

        isExerciseStarted = false;
        isExerciseStopping = false;
        SetBrakes(false, false);
        OnExerciseStop?.Invoke();
        onComplete?.Invoke();
        loader.SetActive(false);
        Time.timeScale = 1f;
        DOTween.PlayAll();

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
                SetBrakes(false, false);
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
        for (int i = 0; i < MaxAttempts; i++)
        {
            bool success = distalRobot.ControlBrakes(unlockRadial, unlockRotational);

            if (success)
            {
                onComplete?.Invoke();
                break;
            }
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

    private IEnumerator MotionRoutineRadialRHB(float target, UnityAction onComplete)
    {
        isMoving = true;
        SetGain(0f, 0f);

        System.Diagnostics.Stopwatch stopwatch = new();
        stopwatch.Start();

        float init_position = DistalData.PositionR;
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

    private IEnumerator MotionRoutineRotationalRHB(float target, UnityAction onComplete)
    {
        isRotating = true;
        SetGain(0f, 0f);

        System.Diagnostics.Stopwatch stopwatch = new();
        stopwatch.Start();

        float pos_phi_init = DistalData.PositionP;
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
                    DistalData.PositionR, current_target,
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
                DistalData.PositionR, target,
                K_STIFF_RADIAL_WALL, K_STIFF_ROT_WALL,
                B_DAMP_RADIAL_WALL, B_DAMP_ROT_WALL,
                1, 1);

        stopwatch.Stop();
        // SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
        isRotating = false;
        onComplete?.Invoke();
    }

    private void MotionRoutineRHBSimple(float pos_rad_targ)
    {
        const int DT_MOTION_R_MSEC = 2000;
        const int N_STEPS_MOTION_R = 80;

        // Set up baseline haptics: 
        float gain_var = 0f;

        for (int i = 0; i <= N_STEPS_MOTION_R; i++)
        {
            // Note use of baseline impedance:
            distalRobot.HL_SetTarget(
                IDX_TARG_BASE,
                pos_rad_targ, 0f,
                K_STIFF_RADIAL_WALL, K_STIFF_ROT_WALL,
                B_DAMP_RADIAL_WALL, B_DAMP_ROT_WALL,
                gain_var, gain_var);

            Thread.Sleep(DT_MOTION_R_MSEC / N_STEPS_MOTION_R);

            gain_var += 1f / N_STEPS_MOTION_R;
        }
    }

    private void MotionRoutineRadialRHBBaseline()
    {
        const int DT_MOTION_BASE_MSEC = 1000;
        const int N_STEPS_MOTION_BASE = 40;

        float gain_var = 0f;

        for (int i = 0; i <= N_STEPS_MOTION_BASE; i++)
        {
            // Note use of baseline impedance:
            distalRobot.HL_SetTarget(
                IDX_TARG_BASE,
                POS_RADIAL_BASE_THROT, POS_ROT_BASE_STEER,
                K_STIFF_RADIAL_BASE_THROT, K_STIFF_ROT_BASE_STEER,
                B_DAMP_RADIAL_BASE_THROT, B_DAMP_ROT_BASE_STEER,
                gain_var, 1f);

            Thread.Sleep(DT_MOTION_BASE_MSEC / N_STEPS_MOTION_BASE);

            gain_var += 1f / N_STEPS_MOTION_BASE;
        }
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
        CmdSetTargetSteerLimit();
    }

   
    #endregion
}