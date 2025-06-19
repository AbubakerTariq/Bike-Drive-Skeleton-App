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

        debugText.text = $"Track length: {track.GetTrackLength().ToString("F2")}m" +
                        $"\nBike coordinates: {bike.GetBikePosition().ToString("F2")}" +
                        $"\nBike cartesian position: {track.GetDistanceAtPosition(bike.GetBikePosition()).ToString("F2")}m" +
                        $"\nBike direction vector: {bike.GetBikeDirectionVector().ToString("F2")}" + 
                        $"\nBike velocity vector: {bike.GetBikeVelocityVector().ToString("F2")}" +
                        $"\nClosest centerline position: {track.GetClosestPointOnCenterLine(bike.GetBikePosition()).ToString("F2")}" + 
                        $"\nTrack curvature at bike position: {track.GetCurvatureAtPosition(bike.GetBikePosition()).ToString("F3")}" +
                        $"\nCenterline tangent direction at bike position: {track.GetTangentAtPosition(bike.GetBikePosition()).ToString("F2")}" +
                        $"\nCenterline tangent angle at bike position: {track.GetTangentAngleAtPosition(bike.GetBikePosition()).ToString("F2")}°";
    }
}