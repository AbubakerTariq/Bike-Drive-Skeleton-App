using System;
using System.IO;
using System.Timers;
using UnityEngine;

public class DataManager : MonoBehaviour
{
    private string filePath;
    
    // The frequency at which data will be stored in the CSV file per second.
    private int frequency = 60;
    
    // The timer to help save the data in the CSV file
    private Timer dataTimer;

    private void Start()
    {
        // The first parameter of the Path.Combine function takes in the directory and the second takes in the File name.
        // Set up the path name according to your needs.
        filePath = Path.Combine(Application.persistentDataPath, "data.csv");

        // The headings to be set up in the CSV file
        string[] headers = new[] { "Date Time", "PositionR", "VelocityR", "PositionP", "VelocityP", "Cartesian Position", "Direction Angle", "Speed", "Tilt Angle" };

        if (!File.Exists(filePath))
        {
            string headerLine = string.Join(",", headers);
            File.WriteAllText(filePath, $"{headerLine}\n");
        }

        ReHandyBotController.instance.OnExerciseStart += StartDataRecording;
        ReHandyBotController.instance.OnExerciseStop += StopDataRecording;
    }

    private void StartDataRecording()
    {
        float interval = 1000f / frequency;
        
        StopDataRecording();
        dataTimer = new(interval);
        dataTimer.Elapsed += SaveDataEntry;
        dataTimer.AutoReset = true;
        dataTimer.Start();
    }

    private void StopDataRecording()
    {
        dataTimer?.Stop();
        dataTimer?.Dispose();
    }

    private void SaveDataEntry(object sender, ElapsedEventArgs e)
    {
        try
        {
            // Using 0 as dummy values for Cartesian Position, Direction Angle, Speed and Title Angle
            string output = $"{DateTime.Now}," +
                            $"{ReHandyBotController.instance.DistalData.PositionR}," +
                            $"{ReHandyBotController.instance.DistalData.VelocityR}," +
                            $"{ReHandyBotController.instance.DistalData.PositionP}," +
                            $"{ReHandyBotController.instance.DistalData.VelocityP}," +
                            $"{0}," +
                            $"{0}," +
                            $"{0}," +
                            $"{0}";

            File.AppendAllText(filePath, $"{output}\n");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error saving data: {ex.Message}");
        }
    }
}