using DG.Tweening;
using Articares.Distal;
using UnityEngine;
using UnityEngine.Events;
using System.Collections;

public class ReHandyBotController : MonoBehaviour
{
    [Space] [Header("UI")]
    [SerializeField] private GameObject loader;

    // Script instance
    private static ReHandyBotController instance;

    // Control library reference
    private static DistalComm distalRobot;

    // Public RHB info related variables
    public static bool RHBConnected => distalRobot.is_device_connected;
    public static DistalComm.ExerciseData DistalData => distalRobot.DistalData;
    public static float PinchValue
    {
        get
        {
            return Remap(DistalData.PositionR, minPinch, maxPinch, 1f, 0f);
        }
    }
    public static string DeviceID
    {
        get
        {
            distalRobot.ReadDeviceId(out string deviceID);
            return deviceID;
        }
    }
    public static DistalComm.SystemInfo SystemInfo
    {
        get
        {
            distalRobot.ReadSystemInfo(out var systemInfo);
            return systemInfo;
        }
    }

    // Public configurable variables
    public static float minPinch = 0.0146f; // Default min pinch
    public static float maxPinch = 0.0375f; // Default max pinch
    public static float minRotation = -1.5f; // Default min rotation
    public static float maxRotation = 1.5f; // Default max rotation
    public static bool isROMGraspingDefault = false;
    public static bool isROMPronationDefault = false;
    public static bool isROMSupinationDefault = false;
    public static bool strapFingerPanelShown = false;
    public static bool isCalibrated = false;

    // New exercise related variables
    private static bool isSystemStarted = false;
    private static bool isExerciseStarted = false;
    private static bool isExerciseStopping = false;

    // Configuration values;
    private static float radialGain = 9f;
    private static float angularGain = 14f;
    private static bool stability = true;
    private static bool safety = true;
    private static float passiveKr = 5000f;
    private static float passiveKp = 60;
    private static float passiveBr = 6f;
    private static float passiveBp = 0.6f;

    // Constants
    private const int MaxAttempts = 10;
    private const string ServerIP = "192.168.102.1";
    private const int ServerPort = 3002;

    // Misc
    private static Coroutine moveRoutine;
    private static Coroutine rotateRoutine;
    private static bool isMoving = false;
    private static bool isRotating = false;

    #region MonoBehavior Functions
    private void Awake()
    {
        // Singleton logic
        if (instance == null)
        {
            distalRobot = new DistalComm();
            instance = this;
        }
        else
        {
            Destroy(gameObject);
        }
        DontDestroyOnLoad(gameObject);

        // Small delay to make sure ReHandyBotSafetyController script is initialized
        // DOVirtual.DelayedCall(0.1f, () => 
        // {
        //     ReHandyBotSafetyController.OnSafetyTriggered += () => isExerciseStarted = false;
        // });
    }

    private void OnApplicationQuit()
    {
        if (distalRobot == null) return;

        if (isExerciseStarted) distalRobot.StopExercise();
        if (isSystemStarted) distalRobot.StopSystem();
        if (RHBConnected) distalRobot.CloseConnection();

        distalRobot = null;
        instance = null;
    }
    #endregion

    #region RHB control functions
    public static bool EstablishConnection(UnityAction onComplete = null)
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

    public static void StartSystem(UnityAction onComplete = null)
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

    public static void StartExercise(bool unlockPinch, bool unlockRotation, UnityAction onComplete = null)
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

    public static void StopExercise(UnityAction onComplete)
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
        instance.loader.SetActive(true);
        Time.timeScale = 0f;
        DOTween.PauseAll();

        if (isMoving)
        {
            isMoving = false;
            instance.StopCoroutine(moveRoutine);
        }

        if (isRotating)
        {
            isRotating = false;
            instance.StopCoroutine(rotateRoutine);
        }

        SetBrakes(true, true);
        rotateRoutine = instance.StartCoroutine(instance.RotateDistalRoutine(0f, ()=>
        {
            moveRoutine = instance.StartCoroutine(instance.MoveDistalRoutine(maxPinch, () =>
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
                instance.loader.SetActive(false);
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
    public static void SetBrakes(bool unlockPinch, bool unlockRotation, UnityAction onComplete = null)
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

    public static void SetTarget(byte targetIndex, float pinchValue, float rotationValue, float pinchStiffness, float rotationStiffness, float pinchDamping, float rotationDamping, float pinchGain, float rotationGain, UnityAction onComplete = null)
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

    public static void SetGain(float radialGain, float angularGain)
    {
        for (int i = 0; i < MaxAttempts; i++)
        {
            if (distalRobot.SetGain(radialGain, angularGain)) break;
        }
    }

    public static void SetEmptyTarget(UnityAction onComplete = null)
    {
        SetTarget(1, 0.0145f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, onComplete);
    }

    public static void AngularCalibration()
    {
        // no need to do anything at this point
    }

    public static void RadialCalibration()
    {
        distalRobot.Calibration(DistalComm.CalibrationType.AxisCalib);
    }

    public static void AllForceSensorsZeroCalibration()
    {
        distalRobot.Calibration(DistalComm.CalibrationType.AllForceSensorsZeroCalib);

        StartExercise(false, false, () =>
        {
            DOVirtual.DelayedCall(0.1f, () =>
            {
                minPinch = DistalData.PositionR;

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

    public static void MoveDistal(float target, UnityAction onComplete = null)
    {
        if (!isExerciseStarted || isExerciseStopping) return;

        target = Mathf.Clamp(target, 0.0145f, 0.06f);

        if (isMoving)
        {
            isMoving = false;
            instance.StopCoroutine(moveRoutine);
        }

        moveRoutine = instance.StartCoroutine(instance.MoveDistalRoutine(target, onComplete));
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
    
    public static void RotateDistal(float target, UnityAction onComplete = null)
    {
        if (!isExerciseStarted || isExerciseStopping) return;

        target = Mathf.Clamp(target, -Mathf.PI / 2f, Mathf.PI / 2f);

        if (isRotating)
        {
            isRotating = false;
            instance.StopCoroutine(rotateRoutine); 
        }

        rotateRoutine = instance.StartCoroutine(instance.RotateDistalRoutine(target, onComplete));
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

    public static float Remap(float value, float from1, float to1, float from2, float to2)
    {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }
    #endregion
}