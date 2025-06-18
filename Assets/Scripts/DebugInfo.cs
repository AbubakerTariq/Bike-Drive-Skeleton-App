using UnityEngine;
using TMPro;

public class DebugInfo : MonoBehaviour
{
    [SerializeField] private TMP_Text debugText;
    private void Update()
    {
        if (Track.Instance == null || MotorbikeController.Instance == null)
            return;

        Track track = Track.Instance;
        MotorbikeController bike = MotorbikeController.Instance;

        debugText.text = $"Track length: {track.GetTrackLength().ToString("F3")}" +
                        $"\nBike coordinates: {bike.GetBikePosition().ToString("F3")}" +
                        $"\nBike cartesian position: {track.GetDistanceAtPosition(bike.GetBikePosition()).ToString("F3")}" +
                        $"\nBike direction vector: {bike.GetBikeDirectionVector().ToString("F3")}" + 
                        $"\nBike velocity vector: {bike.GetBikeVelocityVector().ToString("F3")}" +
                        $"\nClosest centerline position: {track.GetClosestPointOnCenterLine(bike.GetBikePosition()).ToString("F3")}" + 
                        $"\nTrack curvature at bike position: {track.GetCurvatureAtPosition(bike.GetBikePosition()).ToString("F3")}" +
                        $"\nTrack tangent direction at bike position: {track.GetTangentAtPosition(bike.GetBikePosition()).ToString("F3")}" +
                        $"\nTrack tangent angle at bike position: {track.GetTangentAngleAtPosition(bike.GetBikePosition()).ToString("F3")}";
    }
}