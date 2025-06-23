// using Articares.Distal;
using System;
using System.IO;
using System.Runtime.InteropServices;
using UnityEngine;
// using static Articares.Distal.DistalComm;

public class ExternalConsoleLogger : MonoBehaviour
{
    // Import AttachConsole to attach to an existing console or create a new one
    [DllImport("kernel32.dll", SetLastError = true, ExactSpelling = true)]
    private static extern bool AttachConsole(int dwProcessId);

    // Constants for attaching to the parent process's console
    private const int ATTACH_PARENT_PROCESS = -1;

    // Import AllocConsole to create a new console
    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool AllocConsole();

    // Import FreeConsole to release the console when done
    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool FreeConsole();

    void Start()
    {
// #if !UNITY_EDITOR
        // Try to attach to the parent process's console
        if (!AttachConsole(ATTACH_PARENT_PROCESS))
        {
            // If attaching fails, allocate a new console
            AllocConsole();
        }

        // Redirect the output streams to the console
        StreamWriter standardOutput = new(Console.OpenStandardOutput()) { AutoFlush = true };
        Console.SetOut(standardOutput);

        StreamWriter standardError = new(Console.OpenStandardError()) { AutoFlush = true };
        Console.SetError(standardError);
// #endif
    }

    public static void Log(string str)
    {
        // Console.WriteLine("[" + DateTime.Now + "] " + str);
        Console.WriteLine(str);
        Debug.Log(str);
    }
}