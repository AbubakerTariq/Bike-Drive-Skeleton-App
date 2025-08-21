using UnityEngine;
using TMPro;

public class DebugInfo : MonoBehaviour
{
    [SerializeField] private TMP_Text debugText;
    private void Update()
    {
        if (Track.instance == null || MotorbikeController.instance == null)
            return;

        Track track = Track.instance;
        MotorbikeController bike = MotorbikeController.instance;
        Vector3 bikePos = bike.GetBikePosition();

        debugText.text = $"Track length: {track.GetTrackLength().ToString("F2")}m" +
                        $"\nBike coordinates: {bike.GetBikePosition().ToString("F2")}" +
                        $"\nBike cartesian position: {track.GetDistanceAtPosition(bikePos).ToString("F2")}m" +
                        $"\nBike direction vector: {bike.GetBikeDirectionVector().ToString("F2")}" + 
                        $"\nBike velocity vector: {bike.GetBikeVelocityVector().ToString("F2")}" +
                        $"\nClosest centerline position: {track.GetClosestPointOnCenterLine(bikePos).ToString("F2")}" + 
                        $"\nTrack curvature at bike position: {track.GetCurvatureAtPosition(bikePos).ToString("F3")}" +
                        $"\nCenterline tangent direction at bike position: {track.GetTangentAtPosition(bikePos).ToString("F2")}" +
                        $"\nCenterline tangent angle at bike position: {track.GetTangentAngleAtPosition(bikePos).ToString("F2")}°";
    }
}