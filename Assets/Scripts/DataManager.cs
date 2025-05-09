using System;
using System.IO;
using System.Timers;
using System.Threading;
using UnityEngine;

public class DataManager : MonoBehaviour
{
    // Variable to store file path of the data file
    private string filePath;
    
    // The thread and timer to help save the data in the CSV file
    private System.Timers.Timer dataTimer;
    private Thread timerThread;

    private void Start()
    {
        SetupDataFile();
        SetupRecordingEvents();
    }

    private void SetupDataFile()
    {
        // The first parameter of the Path.Combine function takes in the directory and the second takes in the File name.
        // Set up the path name according to your needs.
        filePath = Path.Combine(Application.persistentDataPath, "data.csv");

        // The headings to be set up in the CSV file
        string[] headers = new[] { "Date Time", "Uptime", "PositionR", "VelocityR", "PositionP", "VelocityP", "Cartesian Position", "Direction Angle", "Speed", "Tilt Angle" };

        if (!File.Exists(filePath))
        {
            string headerLine = string.Join(",", headers);
            File.WriteAllText(filePath, $"{headerLine}\n");
        }
    }

    private void SetupRecordingEvents()
    {
        ReHandyBotController.instance.OnExerciseStart += StartDataRecording;
        ReHandyBotController.instance.OnExerciseStop += StopDataRecording;
    }

    // This is for usage for SetOffsetForces command, currently being called with dummy values
    private void SetOffsetForces()
    {
        ReHandyBotController.instance.SetOffsetForces(0f, 0f);
    }

    private void Destroy()
    {
        StopDataRecording();
    }

    private void OnApplicationQuit()
    {
        StopDataRecording();
    }

    private void StartDataRecording()
    {
        StopDataRecording();

        timerThread = new Thread(() =>
        {
            dataTimer = new(1f);
            dataTimer.Elapsed += OnTimerElapsed;
            dataTimer.AutoReset = true;
            dataTimer.Start();
        });
        timerThread.Start();
    }

    private void StopDataRecording()
    {
        timerThread?.Abort();
        dataTimer?.Stop();
        dataTimer?.Dispose();
    }

    private int counter = 0;

    private void OnTimerElapsed(object sender, ElapsedEventArgs e)
    {
        if (counter > 0)
        {
            if (counter >= 5)
            {
                counter = 0;
            }
            else
            {
                counter++;
                return;
            }
        }

        // Passing 0 as dummy values for Cartesian Position, Direction Angle, Speed, and Tilt Angle
        SaveDataEntry(0f, 0f, 0f, 0f);
        counter++;
    }

    private readonly object fileLock = new object();
    private void SaveDataEntry(float cartesianPosition, float directionAngle, float speed, float tiltAngle)
    {
        try
        {
            string datetime = DateTime.UtcNow.ToLocalTime().ToString("MMM-dd-yyyy HH:mm:ss.fff tt \"GMT\"zzz") ;
            string output = $"{datetime}," +
                            $"{ReHandyBotController.instance.DistalData.UptimeMs}," +
                            $"{ReHandyBotController.instance.DistalData.PositionR}," +
                            $"{ReHandyBotController.instance.DistalData.VelocityR}," +
                            $"{ReHandyBotController.instance.DistalData.PositionP}," +
                            $"{ReHandyBotController.instance.DistalData.VelocityP}," +
                            $"{cartesianPosition}," +
                            $"{directionAngle}," +
                            $"{speed}," +
                            $"{tiltAngle}";
            lock (fileLock)
            {
                File.AppendAllText(filePath, $"{output}\n");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error saving data: {ex.Message}");
        }
    }
}