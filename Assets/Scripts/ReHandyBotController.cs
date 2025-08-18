using Articares.Distal;
using DG.Tweening;
// using UnityEngine.UI;
// using UnityEngine.Video;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
//using System.Runtime.Remoting.Messaging;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.SceneManagement;
using System.Timers;

public class ReHandyBotController : MonoBehaviour
{
    ////////////////////////////////////////////////////////////////////////////
    // Real-time steps (CRITICAL):
    ////////////////////////////////////////////////////////////////////////////

    // Application time step:
    public const int DT_STEP_APP_MSEC = 50;  
    // public const int DECIM_STEP_CTRL = 10; // removed 18.08.2025

    // Set Target command time step:
    public const int DT_STEP_SET_TARG_MSEC = 1000;
    // public const int DECIM_STEP_SET_TARG = 200; // removed 18.08.2025

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

    private float K_STIFF_RADIAL_WALL = 5000f;
    private float K_STIFF_ROT_WALL = 1.2f;

    private float B_DAMP_RADIAL_WALL = 30f;
    private float B_DAMP_ROT_WALL = 0.092f;

    private float POS_RADIAL_MIN = 0.0145f;
    private float POS_RADIAL_MAX = 0.06f;

    private float POS_ROT_MIN = -Mathf.PI / 2f;
    private float POS_ROT_MAX = Mathf.PI / 2f;

    ////////////////////////////////////////////////////////////////////////////
    // RHB control settings - CRITICAL
    // NOTE: use [RHB ctrl params - stability v5b game settings 4-axis.xlsx] to calculate damping as a function of stiffness
    ////////////////////////////////////////////////////////////////////////////

    // Throttle - default haptics settings:
    [HideInInspector] public float POS_RADIAL_THROT_ZERO = 0.029f;
    private float K_STIFF_RADIAL_THROT = 5000f; // 2500f;
    private float B_DAMP_RADIAL_THROT = 45f; //21.0f;

    // Steering - default haptics settings:
    [HideInInspector] public float POS_ROT_STEER_ZERO = 0f;
    private float K_STIFF_ROT_STEER = 0.1f; // 0.15f; 
    private float B_DAMP_ROT_STEER = 0.015f; // 0.0185f;

    ////////////////////////////////////////////////////////////////////////////
    // Impedance for RHB motion limits:
    ////////////////////////////////////////////////////////////////////////////   

    private float K_STIFF_ROT_LIMIT = 0; // 1.2f;
    private float B_DAMP_ROT_LIMIT = 0; // 21f;

    public float ANGLE_ROT_LIM_DEG = 20f;

    ////////////////////////////////////////////////////////////////////////////
    // Target indices:
    ////////////////////////////////////////////////////////////////////////////

    private byte IDX_TARG_STEER = 1;
    private byte IDX_TARG_LIM = 2;

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

    private bool stability = true;
    private bool safety = true;

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
    // Misc:
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

    [HideInInspector] public int step_count = 0;

    ////////////////////////////////////////////////////////////////////////////
    // Thread and timer dor SetTarget process:
    ////////////////////////////////////////////////////////////////////////////

    private System.Timers.Timer timerSetTarget;
    private Thread threadTimerSetTarget;

    ////////////////////////////////////////////////////////////////////////////
    // Data display:
    ////////////////////////////////////////////////////////////////////////////

    private int DECIM_STEP_DISP_DATA = 200;
    private int DT_DISP_DATA_MSEC;

    // private bool DISP_TIMER_ACTIVITY_ON = true;
    private bool DISP_UPDATE_ON = false;

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

        // Set up time steps: 
        // DT_STEP_CTRL_MSEC     = DECIM_STEP_CTRL       * DataManager.DT_STEP_DATA_FBK_MSEC;  // removed 18.08.2025
        // DT_STEP_SET_TARG_MSEC = DECIM_STEP_SET_TARG   * DataManager.DT_STEP_DATA_FBK_MSEC;  // removed 18.08.2025
        DT_DISP_DATA_MSEC     = DECIM_STEP_DISP_DATA * DataManager.DT_STEP_DATA_FBK_MSEC;
}

private void Start()
    {
        // Reset Ethernet port to prevent those frequent connection delays:
        System.Diagnostics.Process.Start("ethernet_reset.bat");

        // Start is only called once as this is a singleton object so we will only connect once at the beginning
        ConnectRHB();    
        
        // Setup Set Target events process:
        SetupSetTargetEvents();
    }

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
        
        /*
        // Conditional thread sleep:
        timerLocked = true;

        while (timerLocked)
        {
            System.Threading.Thread.Sleep(DT_STEP_MSEC);
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
        TimeSpan timeElapsed = TimeSpan.FromSeconds(timeElapsedValue);

        // Time elapsed dispay:
        string timeElapsedText = String.Format("{0:#00}", timeElapsed.Minutes) + ":" + String.Format("{0:#00}", timeElapsed.Seconds);

        // Display section:
        if ((step_count % DECIM_DATA_DISP_RHB_CTRL) == 0 && DISP_TIMER_ACTIVITY_ON)
            ExternalConsoleLogger.Log("Update(" + step_count + "): time elapsed [" + timeElapsedText + "]\n");

        ////////////////////////////////////////////////////////////////////////////
        // Update step counter:
        ////////////////////////////////////////////////////////////////////////////
        
        step_count++;

        ////////////////////////////////////////////////////////////////////////////
        // Using an action queue to perform Unity related tasks (i.e UI changes) which are not allowed to be done from a background thread
        ////////////////////////////////////////////////////////////////////////////
    
        // while (MainThreadActionQueue.Count > 0)
        //    MainThreadActionQueue.Dequeue().Invoke();
        */
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
            distalRobot.SetSafety(safety);
            onComplete?.Invoke();
            return;
        }

        for (int i = 0; i < MaxAttempts; i++)
        {
            bool success = distalRobot.StartSystem();

            if (success)
            {
                distalRobot.SetSafety(safety);
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
        {
            StopExercise();
        }
        else
        {
            // StartExercise(true, true);
            
            StartExercise(true, true, () =>
            {
               

                SetTargetValidated(IDX_TARG_STEER,
                    POS_RADIAL_THROT_ZERO, POS_ROT_STEER_ZERO,
                    K_STIFF_RADIAL_THROT, K_STIFF_ROT_STEER,
                    B_DAMP_RADIAL_THROT, B_DAMP_ROT_STEER,
                    1.0f, 1.0f);
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

            success_set_targ_empty = HL_SetTargetEmpty(); // was SetTargetValidatedEmpty();

            onComplete?.Invoke();
            
            if (isCalibrated) 
                OnExerciseStart?.Invoke();

            return;
        }

        for (int i = 0; i < MaxAttempts; i++)
        {
            distalRobot.HL_StartExercise(1, unlockRadial, unlockRotational, 0f, 0f, out bool startExerciseResponse, out bool setGainResponse, FORCE_GAIN_RADIAL, FORCE_GAIN_ROT, stability);

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
        
        motionRoutineRotational = StartCoroutine(motionRoutineRotationalRHB(0f, ()=>
        {
            motionRoutineRadial = StartCoroutine(motionRoutineRadialRHB(POS_RADIAL_MIN, () =>
            {
                for (int i = 0; i < MaxAttempts; i++)
                {
                    if (distalRobot.StopExercise())
                        break;

                    if (distalRobot.LastErrorMessage.Contains("Timeout while waiting for StopExercise response"))
                        continue;
                }

                HL_SetTargetEmpty();

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
    
    /*
    private bool SetTargetValidatedEmpty(UnityAction onComplete = null)
    {
        return SetTargetValidated(IDX_TARG_STEER, POS_RADIAL_MIN, 0f, 0f, 0f, 0f, 0f, 0f, 0f, onComplete);
    }
    */

    private bool HL_SetTargetEmpty(UnityAction onComplete = null)
    {
        return distalRobot.HL_SetTarget(IDX_TARG_STEER, POS_RADIAL_MIN, 0f, 0f, 0f, 0f, 0f, 0f, 0f);
    }

    private void SetGain(float radialGain, float angularGain)
    {
        for (int i = 0; i < MaxAttempts; i++)
        {
            if (distalRobot.SetGain(radialGain, angularGain)) break;
        }
    }

    public void SetOffsetForces(float radialOffsetForce, float angularOffsetForce)
    {
        for (int i = 0; i < MaxAttempts; i++)
        {
            if (distalRobot.SetOffsetForces(radialOffsetForce, angularOffsetForce)) break;
        }
    }

    private void MoveDistal(float target, UnityAction onComplete = null)
    {
        if (!isExerciseStarted || isExerciseStopping) return;

        target = Mathf.Clamp(target, POS_RADIAL_MIN, POS_RADIAL_MAX);

        if (isMoving)
        {
            isMoving = false;
            StopCoroutine(motionRoutineRadial);
        }

        motionRoutineRadial = StartCoroutine(motionRoutineRadialRHB(target, onComplete));
    }

    private IEnumerator motionRoutineRadialRHB(float target, UnityAction onComplete)
    {
        isMoving = true;
        SetGain(0f, 0f);

        float Kr = K_STIFF_RADIAL_WALL;
        float Kp = K_STIFF_ROT_WALL;
        float Br = B_DAMP_RADIAL_WALL;
        float Bp = B_DAMP_ROT_WALL;
        float dt_interval_msec = DataManager.DT_STEP_DATA_FBK_MSEC;

        System.Diagnostics.Stopwatch stopwatch = new();
        stopwatch.Start();

        float init_position = DistalData.PositionR;
        float current_target = init_position;
        float current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;
        float init_time_ms = current_time_ms;
        float prev_time_ms = current_time_ms;
        float speed_factor = 1f;

        while (((init_position < target) && (current_target < target)) || ((init_position >= target) && (current_target > target)))
        {
            current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;

            if ((current_time_ms - prev_time_ms) >= dt_interval_msec)
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

                distalRobot.HL_SetTarget(IDX_TARG_STEER, current_target, 0, Kr, Kp, Br, Bp, 1, 1);

                // Removed SetTargetValidated() routine (15.08.2025):
                /*
                if (isExerciseStopping)
                {
                    distalRobot.HL_SetTarget(IDX_TARG_STEER, current_target, 0, Kr, Kp, Br, Bp, 1, 1);
                }
                else
                {
                    SetTargetValidated(IDX_TARG_STEER, current_target, 0, Kr, Kp, Br, Bp, 1, 1);
                }
                */

                prev_time_ms = current_time_ms;
            }
            yield return null;
        }

        if (!isExerciseStopping)
        {
            // SetTargetValidated(IDX_TARG_STEER, target, 0, Kr, 0, Br, 0, 1, 1);
            distalRobot.HL_SetTarget(IDX_TARG_STEER, target, 0, Kr, 0, Br, 0, 1, 1);
        }

        stopwatch.Stop();
        SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
        isMoving = false;
        onComplete?.Invoke();
    }

    private IEnumerator motionRoutineRotationalRHB(float target, UnityAction onComplete)
    {
        isRotating = true;
        SetGain(0f, 0f);

        float Kr = K_STIFF_RADIAL_WALL;
        float Kp = K_STIFF_ROT_WALL;
        float Br = B_DAMP_RADIAL_WALL;
        float Bp = B_DAMP_ROT_WALL;
        float loop_interval_ms = DataManager.DT_STEP_DATA_FBK_MSEC;

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

            if ((current_time_ms - prev_time_ms) >= loop_interval_ms)
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

                distalRobot.HL_SetTarget(IDX_TARG_STEER, DistalData.PositionR, current_target, Kr, Kp, Br, Bp, 1, 1);

                // Removed SetTargetValidated() routine (15.08.2025):
                /*
                if (isExerciseStopping)
                {
                    distalRobot.HL_SetTarget(IDX_TARG_STEER, DistalData.PositionR, current_target, Kr, Kp, Br, Bp, 1, 1);
                }
                else
                {
                    SetTargetValidated(IDX_TARG_STEER, DistalData.PositionR, current_target, Kr, Kp, Br, Bp, 1, 1);
                }
                */

                prev_time_ms = current_time_ms;
            }

            yield return null;
        }

        if (!isExerciseStopping) 
        {
            // SetTargetValidated(IDX_TARG_STEER, DistalData.PositionR, target, Kr, Kp, Br, Bp, 1, 1);
            distalRobot.HL_SetTarget(IDX_TARG_STEER, DistalData.PositionR, target, Kr, Kp, Br, Bp, 1, 1);
        }

        stopwatch.Stop();
        SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
        isRotating = false;
        onComplete?.Invoke();
    }
    #endregion

    #region Set Target functions
    private void SetupSetTargetEvents()
    {
        ReHandyBotController.instance.OnExerciseStart += StartSetTargetEvents;
        ReHandyBotController.instance.OnExerciseStop += StopSetTargetEvents;
    }

    // This is for usage for SetOffsetForces command, currently being called with dummy values
    private void SetOffsetForces()
    {
        ReHandyBotController.instance.SetOffsetForces(0f, 0f);
    }

    private void Destroy()
    {
        StopSetTargetEvents();
    }

    private void StartSetTargetEvents()
    {
        StopSetTargetEvents();

        threadTimerSetTarget = new Thread(() =>
        {
            timerSetTarget = new System.Timers.Timer(DT_STEP_SET_TARG_MSEC);
            timerSetTarget.Elapsed += SendCmdSetTarget;
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

    private void SendCmdSetTarget(object sender, ElapsedEventArgs e)
    {
        ////////////////////////////////////////////////////////////////////////////
        // RHB coordinates:
        //////////////////////////////////////////////////////////////////////////// 

        float pos_radial = ReHandyBotController.instance.DistalData.PositionR;
        float pos_phi = ReHandyBotController.instance.DistalData.PositionP;

        ////////////////////////////////////////////////////////////////////////////
        // Compute limiting force for rotation angle:
        ////////////////////////////////////////////////////////////////////////////

        float angle_rot_lim = ANGLE_ROT_LIM_DEG * (float)Math.PI / 180f;

        float k_stiff_radial_lim = 0f;
        float k_stiff_rot_lim;

        float b_damp_radial_lim = 0f;
        float b_damp_rot_lim;

        float pos_radial_lim = POS_RADIAL_MIN;
        float pos_rot_lim;

        float switch_radial = 0f;
        float switch_rot = 1.0f;

        // If rotation limit exceeded, apply nonzero values to rotational siffness and damping:
        if ((pos_phi - angle_rot_lim > 0f) || (pos_phi + angle_rot_lim < 0f))
        {
            k_stiff_rot_lim = K_STIFF_ROT_LIMIT;
            b_damp_rot_lim = B_DAMP_ROT_LIMIT;
        }
        else
        {
            k_stiff_rot_lim = 0f;
            b_damp_rot_lim = 0f;
        }

        if (pos_phi - angle_rot_lim > 0f)
            pos_rot_lim = angle_rot_lim;
        else if (pos_phi + angle_rot_lim < 0f)
            pos_rot_lim = -angle_rot_lim;
        else
            pos_rot_lim = 0f;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        /*
        TargetParams

        Index;
        R;
        P;
        KR;
        KP;
        BR;
        BP;
        AlphaR;
        AlphaP;
        */

        bool success_set_target;

        if (ExerciseActive)
        {
            success_set_target = distalRobot.HL_SetTarget(
                IDX_TARG_STEER, // IDX_TARG_LIM,
                POS_RADIAL_MIN, 0f,
                0f, K_STIFF_ROT_LIMIT,
                0f, B_DAMP_ROT_LIMIT,
                0f, 0f);

            /*
            SetTarget(IDX_TARG_LIM,
                pos_radial_lim, pos_rot_lim,
                k_stiff_radial_lim, k_stiff_rot_lim,
                b_damp_radial_lim, b_damp_rot_lim,
                switch_radial, switch_rot);  
            */

            // Display section:
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("HL_SetTarget() sent - success [" + success_set_target + "]\n");
        }

        ////////////////////////////////////////////////////////////////////////////
        // Display section: 
        ////////////////////////////////////////////////////////////////////////////

        if (ExerciseActive && DISP_UPDATE_ON)
        {
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("[" + step_count + "] timeElapsedValue:[" + timeElapsedValue +
                "]  RHB RADIAL pos [" + String.Format("{0:#0.0000}", pos_radial) +
                "]  ROTATIONAL pos [" + String.Format("{0:#0.00}", pos_phi) +
                "]  stiff ROT limit [" + String.Format("{0:#0.00}", k_stiff_rot_lim) +
                "]  damp ROT limit [" + String.Format("{0:#0.00}", b_damp_rot_lim) +
                "]  pos ROT limit [" + String.Format("{0:#0.00}", pos_rot_lim) +
                "]\n");
        }
    }
    #endregion

    #region Misc functions

    private float Remap(float value, float from1, float to1, float from2, float to2)
    {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }
    #endregion
}