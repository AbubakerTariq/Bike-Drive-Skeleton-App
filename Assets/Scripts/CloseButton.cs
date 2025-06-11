using UnityEngine;

public class CloseButton : MonoBehaviour
{
    public void QuitApplication()
    {
        Debug.Log("Quitting Application");
        Application.Quit();
    }
}