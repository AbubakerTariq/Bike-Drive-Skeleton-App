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
    ///
    public static ReHandyBotController instance;

    ////////////////////////////////////////////////////////////////////////////
    // Control library reference:
    ////////////////////////////////////////////////////////////////////////////
    ///
    private DistalComm distalRobot = new();

    ////////////////////////////////////////////////////////////////////////////
    // RHB info related variables:
    ////////////////////////////////////////////////////////////////////////////
    ///
    private bool RHBConnected => distalRobot.is_device_connected;
    public DistalComm.ExerciseData DistalData => distalRobot.DistalData;
    public bool ExerciseActive => isExerciseStarted;

    ////////////////////////////////////////////////////////////////////////////
    // New exercise related variables:
    ////////////////////////////////////////////////////////////////////////////
    ///
    private bool isSystemStarted = false;
    private bool isExerciseStarted = false;
    private bool isExerciseStopping = false;

    ////////////////////////////////////////////////////////////////////////////
    // Configuration values:
    ////////////////////////////////////////////////////////////////////////////
    ///
    private float radialGain = 9f;
    private float angularGain = 14f;
    private bool stability = true;
    private bool safety = true;
    private float passiveKr = 5000f;
    private float passiveKp = 6f;
    private float passiveBr = 60f;
    private float passiveBp = 0.6f;
    private float minPositionR = 0.0145f;
    private float maxPositionR = 0.06f;

    ////////////////////////////////////////////////////////////////////////////
    // Constants:
    ////////////////////////////////////////////////////////////////////////////
    ///
    private const int MaxAttempts = 10;
    private const string ServerIP = "192.168.102.1";
    private const int ServerPort = 3002;

    ////////////////////////////////////////////////////////////////////////////
    // Misc:
    ////////////////////////////////////////////////////////////////////////////
    ///
    private Queue<Action> MainThreadActionQueue = new();
    private Coroutine moveRoutine;
    private Coroutine rotateRoutine;
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
    // RHB control settings:
    ////////////////////////////////////////////////////////////////////////////

    // Throttle settings:
    public float POS_RAD_THROT_ZERO = 0.029f;
    public float K_STIFF_RAD_THROT  = 1250f;
    public float B_DAMP_RAD_THROT   = 16.5f;

    public float DIST_RAD_THROT_FULL = 0.005f;
    public float INPUT_THROT_MAX = 1.5f;

    // Steering settings:
    public float POS_PHI_STEER_ZERO = 0f;
    public float K_STIFF_PHI_STEER  = 0.3f;
    public float B_DAMP_PHI_STEER   = 0.029f;

    // public float FACT_PHI_STEER = -0.5f;
    // public float INPUT_STEER_MAX = Mathf.PI / 4.0f;
    public float INPUT_STEER_LIM = 3.0f * Mathf.PI / 180f;

    ////////////////////////////////////////////////////////////////////////////
    // Timers & data display:
    ////////////////////////////////////////////////////////////////////////////
    ///
    private bool timerActive = false;
    private bool timerActivePrev = false;
    private bool timerLocked = false;
    private bool timerLockDetected = false;

    private float timeElapsedValue = 0f;

    private int T_TIMER_LOCK_MSEC = 50;
    private const int DECIM_DATA_DISP_RHB_CTRL = 50;

    public int step_count = 0;

    ////////////////////////////////////////////////////////////////////////////
    // Methods section:
    ////////////////////////////////////////////////////////////////////////////
    
    private bool DISP_TIMER_ACTIVITY_ON = true;
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

        while (timerLocked)
        {
            System.Threading.Thread.Sleep(T_TIMER_LOCK_MSEC);
            timerLockDetected = true;
        }

        if (timerLockDetected) {

            if ((step_count % DECIM_DATA_DISP_RHB_CTRL) ==0 && DISP_TIMER_ACTIVITY_ON)
            {
                ExternalConsoleLogger.Log(" ");
                ExternalConsoleLogger.Log("____________________________________________________________________");
                ExternalConsoleLogger.Log("Update(): timerLockDetected = [" + timerLockDetected + "], timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");
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
        // Send target command to RHB firmware:
        ////////////////////////////////////////////////////////////////////////////

        byte IDX_TARG = 1;

        SetTarget(IDX_TARG,
            POS_RAD_THROT_ZERO, POS_PHI_STEER_ZERO,
            K_STIFF_RAD_THROT, K_STIFF_PHI_STEER,
            B_DAMP_RAD_THROT, B_DAMP_PHI_STEER,
            1.0f, 1.0f);

        ////////////////////////////////////////////////////////////////////////////
        // Console output:
        ////////////////////////////////////////////////////////////////////////////
        
        float pos_rad = ReHandyBotController.instance.DistalData.PositionR;
        float pos_phi = ReHandyBotController.instance.DistalData.PositionP;

        if ((step_count % DECIM_DATA_DISP_RHB_CTRL) == 0 && ExerciseActive && DISP_UPDATE_ON) {
            ExternalConsoleLogger.Log("____________________________________________________________________");
            ExternalConsoleLogger.Log("[" + step_count + "] timeElapsedValue:[" + timeElapsedValue + "] pos[" +
                String.Format("{0:#0.0000}", pos_rad) + "][" +
                String.Format("{0:#0.00}", pos_phi) + 
                "]\n");
        }

        ////////////////////////////////////////////////////////////////////////////
        // Update step counter:
        ////////////////////////////////////////////////////////////////////////////
        ///
        step_count++;

        ////////////////////////////////////////////////////////////////////////////
        // Using an action queue to perform Unity related tasks (i.e UI changes) which are not allowed to be done from a background thread
        ////////////////////////////////////////////////////////////////////////////
        
        if (MainThreadActionQueue.Count == 0) return;
        
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
                minPinch = Math.Clamp(minPinch, minPositionR, maxPositionR);

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
            StartExercise(true, true);
        }
    }

    private void StartExercise(bool unlockPinch, bool unlockRotation, UnityAction onComplete = null)
    {
        bool timer_started = false;

        if (isExerciseStarted)
        {

            SetBrakes(unlockPinch, unlockRotation);
            SetEmptyTarget();
            onComplete?.Invoke();
            
            if (isCalibrated) 
                OnExerciseStart?.Invoke();
            
            return;
        }

        for (int i = 0; i < MaxAttempts; i++)
        {
            distalRobot.HL_StartExercise(1, unlockPinch, unlockRotation, 0f, 0f, out bool startExerciseResponse, out bool setGainResponse, radialGain, angularGain, stability);

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
 
            if (isCalibrated)
                OnExerciseStart?.Invoke();
            
            onComplete?.Invoke();

            if (setGainResponse) break;
            SetGain(radialGain, angularGain);
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
        timerLocked = true;
        timerActivePrev = timerActive;
        timerActive = false;
        System.Threading.Thread.Sleep(T_TIMER_LOCK_MSEC);
        timerLocked = false;
        timerLocked = false;

        ExternalConsoleLogger.Log(" ");
        ExternalConsoleLogger.Log("____________________________________________________________________");
        ExternalConsoleLogger.Log("StopExercise(): timerActivePrev [" + timerActivePrev + "], timerActive [" + timerActive + "]\n");

        if (isMoving)
        {
            isMoving = false;
            StopCoroutine(moveRoutine);
        }

        if (isRotating)
        {
            isRotating = false;
            StopCoroutine(rotateRoutine);
        }

        SetBrakes(true, true);
        rotateRoutine = StartCoroutine(RotateDistalRoutine(0f, ()=>
        {
            moveRoutine = StartCoroutine(MoveDistalRoutine(maxPinch, () =>
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
    /// <param name="unlockPinch">Horizontal Axis</param>
    /// <param name="unlockRotation">Vertical Axis</param>
    private void SetBrakes(bool unlockPinch, bool unlockRotation, UnityAction onComplete = null)
    {
        for (int i = 0; i < MaxAttempts; i++)
        {
            bool success = distalRobot.ControlBrakes(unlockPinch, unlockRotation);

            if (success)
            {
                onComplete?.Invoke();
                break;
            }
        }

        onComplete?.Invoke();
    }

    private void SetTarget(byte targetIndex, float pinchValue, float rotationValue, float pinchStiffness, float rotationStiffness, float pinchDamping, float rotationDamping, float pinchGain, float rotationGain, UnityAction onComplete = null)
    {
        if (!isExerciseStarted || isExerciseStopping) return;
        
        pinchValue = Mathf.Clamp(pinchValue, 0.0145f, 0.06f);
        rotationValue = Mathf.Clamp(rotationValue, -Mathf.PI / 2f, Mathf.PI / 2f);

        for (int i = 0; i < MaxAttempts; i++)
        {
            bool success = distalRobot.HL_SetTarget(targetIndex, pinchValue, rotationValue, pinchStiffness, rotationStiffness, pinchDamping, rotationDamping, pinchGain, rotationGain);

            if (success)
            {
                onComplete?.Invoke();
                break;
            }
        }
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
            StopCoroutine(moveRoutine);
        }

        moveRoutine = StartCoroutine(MoveDistalRoutine(target, onComplete));
    }

    private IEnumerator MoveDistalRoutine(float target, UnityAction onComplete)
    {
        isMoving = true;
        SetGain(0f, 0f);

        float Kr = passiveKr;
        float Kp = passiveKp;
        float Br = passiveBr;
        float Bp = passiveBp;
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
        SetGain(radialGain, angularGain);
        isMoving = false;
        onComplete?.Invoke();
    }
    
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

    private IEnumerator RotateDistalRoutine(float target, UnityAction onComplete)
    {
        isRotating = true;
        SetGain(0f, 0f);

        float Kr = passiveKr;
        float Kp = passiveKp;
        float Br = passiveBr;
        float Bp = passiveBp;
        float loop_interval_ms = 1f / 200f * 1000f;

        System.Diagnostics.Stopwatch stopwatch = new();
        stopwatch.Start();

        float init_position = DistalData.PositionP;
        float current_target = init_position;
        float current_time_ms = (float)stopwatch.Elapsed.TotalMilliseconds;
        float init_time_ms = current_time_ms;
        float prev_time_ms = current_time_ms;
        float speed_factor = 0.75f;
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
        SetGain(radialGain, angularGain);
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