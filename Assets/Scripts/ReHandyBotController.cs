using DG.Tweening;
using Articares.Distal;
using UnityEngine;
using UnityEngine.Events;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using TMPro;

public class ReHandyBotController : MonoBehaviour
{
    [Space] [Header("UI")]
    [SerializeField] private GameObject loader;
    [SerializeField] private TMP_Text loaderText;

    // Script instance
    public static ReHandyBotController instance;

    // Control library reference
    private DistalComm distalRobot = new();

    // RHB info related variables
    private bool RHBConnected => distalRobot.is_device_connected;
    private DistalComm.ExerciseData DistalData => distalRobot.DistalData;

    // New exercise related variables
    private bool isSystemStarted = false;
    private bool isExerciseStarted = false;
    private bool isExerciseStopping = false;

    // Configuration values;
    private float radialGain = 9f;
    private float angularGain = 14f;
    private bool stability = true;
    private bool safety = true;
    private float passiveKr = 5000f;
    private float passiveKp = 60;
    private float passiveBr = 6f;
    private float passiveBp = 0.6f;
    private float minPositionR = 0.0145f;
    private float maxPositionR = 0.06f;

    // Constants
    private const int MaxAttempts = 10;
    private const string ServerIP = "192.168.102.1";
    private const int ServerPort = 3002;

    // Misc
    private Coroutine moveRoutine;
    private Coroutine rotateRoutine;
    private bool isMoving = false;
    private bool isRotating = false;
    private float minPinch = 0.0145f;
    private float maxPinch = 0.0375f;
    private Thread connectionThread;
    private Tween connectionTween;
    private bool allowCalibration = false;
    private Queue<Action> MainThreadActionQueue = new();

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
    }

    private void Start()
    {
        // Start is only called once as this is a singleton object so we will only connect once at the beginning
        ConnectRHB();
    }

    private void Update()
    {
        if (allowCalibration && Input.GetKeyDown(KeyCode.Y))
            Calibrate(OnCalibrate);

        if (MainThreadActionQueue.Count == 0) return;
        
        while (MainThreadActionQueue.Count > 0)
            MainThreadActionQueue.Dequeue().Invoke();
    }

    private void OnApplicationQuit()
    {
        connectionThread?.Abort();
        connectionTween?.Kill();

        if (distalRobot == null)
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
        loaderText.text = "Align grippers horizontally and close the grippers\nPress Y to calibrate";
        allowCalibration = true;
    }

    private void OnCalibrate()
    {
        loader.SetActive(false);
        allowCalibration = false;

        StartExercise(false, false, () =>
        {
            DOVirtual.DelayedCall(0.1f, () =>
            {
                minPinch = DistalData.PositionR;
                minPinch = Math.Clamp(minPinch, minPositionR, maxPositionR);

                for (int i = 0; i < MaxAttempts; i++)
                {
                    bool success = distalRobot.StopExercise();

                    if (success)
                    {
                        SetBrakes(false, false);
                        isExerciseStarted = false;
                        break;
                    }
                }
            });
        });
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

    private void StartExercise(bool unlockPinch, bool unlockRotation, UnityAction onComplete = null)
    {
        if (isExerciseStarted)
        {
            SetBrakes(unlockPinch, unlockRotation);
            SetEmptyTarget();
            onComplete?.Invoke();
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
            onComplete?.Invoke();

            if (setGainResponse) break;
            SetGain(radialGain, angularGain);
            break;
        }
    }

    private void StopExercise(UnityAction onComplete)
    {
        if (!isExerciseStarted)
        {
            SetBrakes(false, false);
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

    private void SetGain(float radialGain, float angularGain)
    {
        for (int i = 0; i < MaxAttempts; i++)
        {
            if (distalRobot.SetGain(radialGain, angularGain)) break;
        }
    }

    private void SetEmptyTarget(UnityAction onComplete = null)
    {
        SetTarget(1, 0.0145f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, onComplete);
    }

    private void Calibrate(UnityAction onComplete = null)
    {
        for (int i = 0; i < MaxAttempts; i++)
        {
            if (distalRobot.Calibration(DistalComm.CalibrationType.AxisCalib)) break;
        }

        for (int i = 0; i < MaxAttempts; i++)
        {
            if (distalRobot.Calibration(DistalComm.CalibrationType.AllForceSensorsZeroCalib)) break;
        }

        onComplete.Invoke();
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

        float init_position = distalRobot.DistalData.PositionR;
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

        float init_position = distalRobot.DistalData.PositionP;
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
                    distalRobot.HL_SetTarget(1, 0.0145f, current_target, Kr, Kp, Br, Bp, 1, 1);
                }
                else
                {
                    SetTarget(1, 0.0145f, current_target, Kr, Kp, Br, Bp, 1, 1);
                }

                prev_time_ms = current_time_ms;
            }

            yield return null;
        }

        if (!isExerciseStopping)
        {
            SetTarget(1, 0.0145f, target, 0, Kp, 0, Bp, 1, 1);
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