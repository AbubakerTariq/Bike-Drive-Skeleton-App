using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(CurvatureCalculator))]
public class CurvatureCalculatorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        CurvatureCalculator calculator = (CurvatureCalculator)target;

        base.OnInspectorGUI();

        if (calculator.curvatures != null)
        {
            EditorGUILayout.LabelField("Curvature Values:");
            for (int i = 0; i < calculator.curvatures.Length; i++)
            {
                EditorGUILayout.LabelField("Curvature " + i + ": " + calculator.curvatures[i].ToString());
            }
        }
    }
}
