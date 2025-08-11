using Articares.Distal;
using DG.Tweening;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.SceneManagement;
// using UnityEngine.UI;
// using UnityEngine.Video;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
//using System.Runtime.Remoting.Messaging;
using System.IO;
using TMPro;
using static Articares.Distal.DistalComm;

public class ReHandyBotController : MonoBehaviour
{
    [Space] [Header("UI")]
    [SerializeField] private GameObject loader;
    [SerializeField] private GameObject exerciseGuidelineText;
    [SerializeField] private TMP_Text loaderText;

    ////////////////////////////////////////////////////////////////////////////
    // Script instance:
    ////////////////////////////////////////////////////////////////////////////
    
    public static ReHandyBotController Instance;

    ////////////////////////////////////////////////////////////////////////////
    // Control library reference:
    ////////////////////////////////////////////////////////////////////////////
    
    private DistalComm distalRobot = new();

    ////////////////////////////////////////////////////////////////////////////
    // RHB info related variables:
    ////////////////////////////////////////////////////////////////////////////
    
    private bool RHBConnected => distalRobot.is_device_connected;
    public DistalComm.ExerciseData DistalData => distalRobot.DistalData;
    public bool ExerciseActive => isExerciseStarted;

    ////////////////////////////////////////////////////////////////////////////
    // New exercise related variables:
    ////////////////////////////////////////////////////////////////////////////
    
    private bool isSystemStarted = false;
    private bool isExerciseStarted = false;
    private bool isExerciseStopping = false;

    ////////////////////////////////////////////////////////////////////////////
    // Configuration values:
    ////////////////////////////////////////////////////////////////////////////
    
    private float FORCE_GAIN_RADIAL = 9f;
    private float FORCE_GAIN_ROT = 14f;

    private float K_STIFF_RADIAL_PASSIVE = 5000f;
    private float K_STIFF_ROT_PASSIVE = 6f; // 11.08.2025: this value is HUGE; when is it actually used??

    private float B_DAMP_RADIAL_PASSIVE = 60f;
    private float B_DAMP_ROT_PASSIVE = 0.6f;

    private float POS_RADIAL_MIN = 0.0145f;
    private float POS_RADIAL_MAX = 0.06f;

    private bool  stability = true;
    private bool  safety = true;

    ////////////////////////////////////////////////////////////////////////////
    // RHB control settings - CRITICAL
    // NOTE: use [RHB ctrl params - stability v5b game settings 4-axis.xlsx] to calculate damping as a function of stiffness
    ////////////////////////////////////////////////////////////////////////////

    // Throttle - default haptics settings:
    [HideInInspector] public float POS_RADIAL_THROT_ZERO = 0.029f;
    private float K_STIFF_RADIAL_THROT = 5000f; // 2500f;
    private float B_DAMP_RADIAL_THROT = 45f; //21.0f;

    // Steering - default haptics settings:
    [HideInInspector] public float POS_PHI_STEER_ZERO = 0f;
    private float K_STIFF_PHI_STEER = 0.1f; // 0.15f;
    private float B_DAMP_PHI_STEER = 0.015f; // 0.0185f;

    public float ANGLE_ROT_MAX_DEG = 20f;

    ////////////////////////////////////////////////////////////////////////////
    // Impedance for motion limits:
    ////////////////////////////////////////////////////////////////////////////   

    private float K_STIFF_ROT_LIMIT = 10f;
    private float B_DAMP_ROT_LIMIT = 21f;

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
    private float minPinch = 0.0145f;
    private float maxPinch = 0.03f;
    private Thread connectionThread;
    private Tween connectionTween;
    private bool isCalibrated = false;
    private bool allowCalibration = false;

    public Action OnExerciseStart;
    public Action OnExerciseStop;

    private const string PrototypeSceneName = "Prototype";

    ////////////////////////////////////////////////////////////////////////////
    // Timers & data display:
    ////////////////////////////////////////////////////////////////////////////

    private bool timerActive = false;
    private bool timerActivePrev = false;
    private bool timerLocked = false;
    private bool timerLockDetected = false;
    private float timeElapsedValue = 0f;

    [HideInInspector] public int DT_TIMER_LOCK_MSEC = 50; // timer lock to control time step

    private const int DECIM_DATA_DISP_RHB_CTRL = 10;

    [HideInInspector] public int step_count = 0;

    ////////////////////////////////////////////////////////////////////////////
    // Methods section:
    ////////////////////////////////////////////////////////////////////////////
    
    private bool DISP_TIMER_ACTIVITY_ON = false;
    private bool DISP_UPDATE_ON = true;

    ////////////////////////////////////////////////////////////////////////////
    // Methods section:
    ////////////////////////////////////////////////////////////////////////////

    #region MonoBehavior Functions

    private void Awake()
    {
        // Singleton logic
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }

        Instance = this;
        DontDestroyOnLoad(gameObject);

        // Set log level so that the logs are stored in Nlog file
        distalRobot.SetLogLevel(DistalComm.DistalLogLevel.Info);
    }

    private void Start()
    {
        // Start is only called once as this is a singleton object so we will only connect once at the beginning
        ConnectRHB();
    }

    private void Update()
    {
        ////////////////////////////////////////////////////////////////////////////
        // State check:
        ////////////////////////////////////////////////////////////////////////////

        // If robot is calibrated and user presses Enter
        // Exercise state will be toggled
        // The exercise will start if it isn't started already
        // The exercise will stop if it is already started
        if (isCalibrated && Input.GetKeyDown(KeyCode.Return))
            ToggleExerciseState();

        ////////////////////////////////////////////////////////////////////////////
        // Allow the user to press Y to calibrate the robot:
        ////////////////////////////////////////////////////////////////////////////

        if (allowCalibration && Input.GetKeyDown(KeyCode.Y))
            Calibrate(OnCalibrate);

        ////////////////////////////////////////////////////////////////////////////
        // Time elapsed display:
        ////////////////////////////////////////////////////////////////////////////    

        /*
        while (timerLocked)
        {
            System.Threading.Thread.Sleep(T_TIMER_LOCK_MSEC);
            timerLockDetected = true;
        }
        */

        // HACK: make thread sleep always:
        System.Threading.Thread.Sleep(DT_TIMER_LOCK_MSEC);
        timerLockDetected = true;

        if (timerLockDetected) {

            if ((step_count % DECIM_DATA_DISP_RHB_CTRL) == 0 && DISP_TIMER_ACTIVITY_ON)
            {
                ExternalConsoleLogger.Log(" ");
                ExternalConsoleLogger.Log("____________________________________________________________________");
                ExternalConsoleLogger.Log("Update(" + step_count +"): timerLockDetected = [" + timerLockDetected + "], timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");
            }

            timerLockDetected = false;
        }

        if (timerActive)
        {
            // Restart timer:
            if (timerActivePrev != timerActive)
            {
                timeElapsedValue = 0f;
            }
            else
            {
                timeElapsedValue += Time.deltaTime;
            }
        }

        // Record timer state for next step:
        timerActivePrev = timerActive;

        // Time elapsed computation:
        TimeSpan timeElapsed = TimeSpan.FromSeconds(timeElapsedValue);

        ////////////////////////////////////////////////////////////////////////////
        // RHB coordinates:
        //////////////////////////////////////////////////////////////////////////// 

        float pos_radial = ReHandyBotController.Instance.DistalData.PositionR;
        float pos_phi = ReHandyBotController.Instance.DistalData.PositionP;

        ////////////////////////////////////////////////////////////////////////////
        // Compute limiting force for rotation angle:
        ////////////////////////////////////////////////////////////////////////////
        
        float angle_rot_max = ANGLE_ROT_MAX_DEG * (float)Math.PI / 180f;

        float k_stiff_radial_lim = 0f;
        float k_stiff_rot_lim;

        float b_damp_radial_lim = 0f;
        float b_damp_rot_lim;

        float pos_radial_lim = 0f;
        float pos_rot_lim;

        float switch_radial = 0f;
        float switch_rot = 1f;

        // If rotation limit exceeded, apply nonzero values to rotational siffness and damping:
        if ((pos_phi - angle_rot_max > 0f) || (pos_phi + angle_rot_max < 0f))
        {
            k_stiff_rot_lim = K_STIFF_ROT_LIMIT;
            b_damp_rot_lim = B_DAMP_ROT_LIMIT;
        }
        else
        {
            k_stiff_rot_lim = 0f;
            b_damp_rot_lim = 0f;
        }

        if (pos_phi - angle_rot_max > 0f)
            pos_rot_lim = angle_rot_max;
        else if (pos_phi + angle_rot_max < 0f)
            pos_rot_lim = -angle_rot_max;
        else
            pos_rot_lim = 0f;

        ////////////////////////////////////////////////////////////////////////////
        // Send limit force commands to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        byte IDX_TARG_LIM = 2;

        /*
        SetTarget(IDX_TARG_LIM,
            pos_radial_lim, pos_rot_lim,
            k_stiff_radial_lim, k_stiff_rot_lim,
            b_damp_radial_lim, b_damp_rot_lim,
            switch_radial, switch_rot);
        */

        SetTarget(IDX_TARG_LIM,
          pos_radial_lim, 0f,
          k_stiff_radial_lim, K_STIFF_ROT_LIMIT,
          b_damp_radial_lim, B_DAMP_ROT_LIMIT,
          switch_radial, 1f);

        ////////////////////////////////////////////////////////////////////////////
        // Console output:
        ////////////////////////////////////////////////////////////////////////////

        if ((step_count % DECIM_DATA_DISP_RHB_CTRL) == 0 && ExerciseActive && DISP_UPDATE_ON) {
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("[" + step_count + "] timeElapsedValue:[" + timeElapsedValue + 
                "]  RHB RADIAL pos [" + String.Format("{0:#0.0000}", pos_radial) + 
                "]  ROTATIONAL pos [" + String.Format("{0:#0.00}", pos_phi) + 
                "]  stiff ROT limit [" + String.Format("{0:#0.00}", k_stiff_rot_lim) + 
                "]  damp ROT limit [" + String.Format("{0:#0.00}", b_damp_rot_lim) + 
                "]  pos ROT limit [" + String.Format("{0:#0.00}", pos_rot_lim) + 
                "]\n");
        }

        ////////////////////////////////////////////////////////////////////////////
        // Update step counter:
        ////////////////////////////////////////////////////////////////////////////
        
        step_count++;

        ////////////////////////////////////////////////////////////////////////////
        // Using an action queue to perform Unity related tasks (i.e UI changes) which are not allowed to be done from a background thread
        ////////////////////////////////////////////////////////////////////////////
        
        if (MainThreadActionQueue.Count == 0) 
            return;
        
        while (MainThreadActionQueue.Count > 0)
            MainThreadActionQueue.Dequeue().Invoke();
    }

    private void OnApplicationQuit()
    {
        // Stop all RHB related processes when the application is closed
        connectionThread?.Abort();
        connectionTween?.Kill();

        if (distalRobot == null || !RHBConnected)
            return;
        
        if (isExerciseStarted) distalRobot.StopExercise();
        if (isSystemStarted) distalRobot.StopSystem();
        if (RHBConnected) distalRobot.CloseConnection();

        distalRobot = null;
        Instance = null;
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
            StartExercise(true, true, () =>
            {
                byte IDX_TARG = 1;
                SetTarget(IDX_TARG,
                    POS_RADIAL_THROT_ZERO, POS_PHI_STEER_ZERO,
                    K_STIFF_RADIAL_THROT, K_STIFF_PHI_STEER,
                    B_DAMP_RADIAL_THROT, B_DAMP_PHI_STEER,
                    1.0f, 1.0f);
            });
        }
    }

    private void StartExercise(bool unlockRadial, bool unlockRotational, UnityAction onComplete = null)
    {
        // bool timer_started = false;

        if (isExerciseStarted)
        {

            SetBrakes(unlockRadial, unlockRotational);
            SetEmptyTarget();
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
                {
                    continue;
                }

                int failureCount = 0;
                while (failureCount < MaxAttempts)
                {
                    if (!distalRobot.HL_SetTarget(1, 0.0145f, 0f, 0f, 0f, 0f, 0f, 0f, 0f))
                    {
                        failureCount++;
                    }
                    else
                    {
                        failureCount = 0;
                        break;
                    }
                }

                if (failureCount >= MaxAttempts)
                {
                    continue;
                }
            }

            isExerciseStarted = true;
            SetEmptyTarget();

            // Start timer:
            /*
            if (!timer_started)
            {
                timerLocked = true;
                timerActivePrev = timerActive;
                timerActive = true;
                System.Threading.Thread.Sleep(T_TIMER_LOCK_MSEC);
                timerLocked = false;

                ExternalConsoleLogger.Log(" ");
                ExternalConsoleLogger.Log("____________________________________________________________________");
                ExternalConsoleLogger.Log("StartExercise(): timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");

                timer_started = true;
            }
            */
 
            if (isCalibrated)
                OnExerciseStart?.Invoke();
            
            onComplete?.Invoke();

            if (setGainResponse) break;
            SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
            break;
        }
    }

    private void StopExercise(UnityAction onComplete = null)
    {
        bool timer_stopped = false;

        if (!isExerciseStarted)
        {
            SetBrakes(false, false);
            OnExerciseStop?.Invoke();
            onComplete?.Invoke();
            return;
        }

        if (isExerciseStopping)
        {
            return;
        }

        isExerciseStopping = true;
        loaderText.text = "Stopping Exercise...";
        loader.SetActive(true);
        Time.timeScale = 0f;
        DOTween.PauseAll();

        // Stop timer:
        /*
        timerLocked = true;
        timerActivePrev = timerActive;
        timerActive = false;
        System.Threading.Thread.Sleep(T_TIMER_LOCK_MSEC);
        timerLocked = false;
        timerLocked = false;

        ExternalConsoleLogger.Log(" ");
        ExternalConsoleLogger.Log("____________________________________________________________________");
        ExternalConsoleLogger.Log("StopExercise(): timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");
        */ 

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
            motionRoutineRadial = StartCoroutine(motionRoutineRadialRHB(maxPinch, () =>
            {
                for (int i = 0; i < MaxAttempts; i++)
                {
                    if (distalRobot.StopExercise())
                    {
                        break;
                    }

                    if (distalRobot.LastErrorMessage.Contains("Timeout while waiting for StopExercise response"))
                    {
                        continue;
                    }

                    int failureCount = 0;
                    while (failureCount < MaxAttempts)
                    {
                        if (!distalRobot.HL_SetTarget(1, 0.0145f, 0f, 0f, 0f, 0f, 0f, 0f, 0f))
                        {
                            break;
                        }
                        else
                        {
                            failureCount++;
                        }
                    }

                    if (failureCount >= MaxAttempts)
                    {
                        break;
                    }
                }

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

    private void SetTarget(byte targetIndex, 
        float radialValue, float rotationValue, 
        float radialStiffness, float rotationStiffness, 
        float radialDamping, float rotationDamping, 
        float radialGain, float rotationGain, UnityAction onComplete = null)
    {
        if (!isExerciseStarted || isExerciseStopping) return;

        radialValue = Mathf.Clamp(radialValue, POS_RADIAL_MIN, POS_RADIAL_MAX);
        rotationValue = Mathf.Clamp(rotationValue, -Mathf.PI / 2f, Mathf.PI / 2f);

        for (int i = 0; i < MaxAttempts; i++)
        {
            bool success = distalRobot.HL_SetTarget(
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
        Debug.Log("Set Target Called: ");
    }

    private void SetEmptyTarget(UnityAction onComplete = null)
    {
        SetTarget(1, 0.0145f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, onComplete);
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

        target = Mathf.Clamp(target, 0.0145f, 0.06f);

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

        float Kr = K_STIFF_RADIAL_PASSIVE;
        float Kp = K_STIFF_ROT_PASSIVE;
        float Br = B_DAMP_RADIAL_PASSIVE;
        float Bp = B_DAMP_ROT_PASSIVE;
        float loop_interval_ms = 1f / 200f * 1000f; //ms

        System.Diagnostics.Stopwatch stopwatch = new();
        stopwatch.Start();

        float init_position = DistalData.PositionR;
        float current_target = init_position;
        float current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;
        float init_time_ms = current_time_ms;
        float prev_time_ms = current_time_ms;
        float speed_factor = 1f;
        int step = 0;

        while (((init_position < target) && (current_target < target)) || ((init_position >= target) && (current_target > target)))
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
                current_target = init_position + (target - init_position) * (10f * Mathf.Pow(t, 3f) - 15f * Mathf.Pow(t, 4f) + 6f * Mathf.Pow(t, 5f));

                // Check if current_target is overshooting actual target
                if (((init_position < target) && (current_target > target)) || ((init_position >= target) && (current_target < target)))
                    current_target = target;

                // Set Updated Target
                DistalComm.Log.Info("MoveDistal() - step " + ++step);
                current_target = Mathf.Clamp(current_target, 0.0145f, 0.06f);
                if (isExerciseStopping)
                {
                    distalRobot.HL_SetTarget(1, current_target, 0, Kr, Kp, Br, Bp, 1, 1);
                }
                else
                {
                    SetTarget(1, current_target, 0, Kr, Kp, Br, Bp, 1, 1);
                }
                prev_time_ms = current_time_ms;
            }
            yield return null;
        }

        if (!isExerciseStopping)
        {
            SetTarget(1, target, 0, Kr, 0, Br, 0, 1, 1);
        }

        stopwatch.Stop();
        SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
        isMoving = false;
        onComplete?.Invoke();
    }
    
    /*
    private void RotateDistal(float target, UnityAction onComplete = null)
    {
        if (!isExerciseStarted || isExerciseStopping) return;

        target = Mathf.Clamp(target, -Mathf.PI / 2f, Mathf.PI / 2f);

        if (isRotating)
        {
            isRotating = false;
            StopCoroutine(rotateRoutine); 
        }

        rotateRoutine = StartCoroutine(RotateDistalRoutine(target, onComplete));
    }
    */
    private IEnumerator motionRoutineRotationalRHB(float target, UnityAction onComplete)
    {
        isRotating = true;
        SetGain(0f, 0f);

        float Kr = K_STIFF_RADIAL_PASSIVE;
        float Kp = K_STIFF_ROT_PASSIVE;
        float Br = B_DAMP_RADIAL_PASSIVE;
        float Bp = B_DAMP_ROT_PASSIVE;
        float loop_interval_ms = 1f / 200f * 1000f;

        System.Diagnostics.Stopwatch stopwatch = new();
        stopwatch.Start();

        float pos_phi_init = DistalData.PositionP;
        float current_target = pos_phi_init;
        float current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;
        float init_time_ms = current_time_ms;
        float prev_time_ms = current_time_ms;
        float speed_factor = 0.75f;
        int step = 0;

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
                DistalComm.Log.Info("RotateDistal() - step " + ++step);
                if (isExerciseStopping)
                {
                    distalRobot.HL_SetTarget(1, DistalData.PositionR, current_target, Kr, Kp, Br, Bp, 1, 1);
                }
                else
                {
                    SetTarget(1, DistalData.PositionR, current_target, Kr, Kp, Br, Bp, 1, 1);
                }

                prev_time_ms = current_time_ms;
            }

            yield return null;
        }

        if (!isExerciseStopping)
        {
            SetTarget(1, DistalData.PositionR, target, Kr, Kp, Br, Bp, 1, 1);
        }

        stopwatch.Stop();
        SetGain(FORCE_GAIN_RADIAL, FORCE_GAIN_ROT);
        isRotating = false;
        onComplete?.Invoke();
    }
    #endregion

    #region Misc functions

    private float Remap(float value, float from1, float to1, float from2, float to2)
    {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }
    #endregion
}