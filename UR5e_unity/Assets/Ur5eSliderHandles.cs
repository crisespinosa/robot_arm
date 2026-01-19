using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.UrdfImporter;

public class Ur5eSliderHandles : MonoBehaviour
{
    [Header("Six sliders (values in degrees)")]
    public Slider q1, q2, q3, q4, q5, q6;

    [Header("Texts (TMP) next to sliders (optional)")]
    public TMP_Text q1Text, q2Text, q3Text, q4Text, q5Text, q6Text;

    [Header("Client that requests PMP trajectory")]
    public Ur5eTrajectoryClientQ client;

    [Header("Apply joints in Unity")]
    public ApplyJointAngles6 applier;

    [Header("URDF joints (for reading initial pose)")]
    public UrdfJointRevolute j1, j2, j3, j4, j5, j6;

    [Header("Behavior")]
    [Tooltip("If true: robot moves immediately while you drag sliders (smooth preview). If false: robot moves ONLY after pressing Send.")]
    public bool previewInUnity = false;

    [Tooltip("If true: set sliders to current robot pose at start (recommended).")]
    public bool syncSlidersOnStart = true;

    // Home pose (rad)
    private float[] homeRad = new float[6];
    private bool homeCaptured = false;

    IEnumerator Start()
    {
        yield return null;
        yield return null;

        if (applier != null && applier.LastQRad != null && applier.LastQRad.Length >= 6)
        {
            SetSlidersFromRad(applier.LastQRad);
            UpdateLabels();
        }
    }


    // Conecta esto a OnValueChanged de cada slider
    public void OnSliderChanged()
    {
        UpdateLabels();

        // SOLO si quieres preview fluido
        if (previewInUnity && applier != null)
            applier.Apply(GetSlidersAsRad());
    }

    // Botón SEND
    public void SendTarget()
    {
        if (client == null)
        {
            Debug.LogError("Ur5eSliderHandles: client is null");
            return;
        }

        // Detener reproducción anterior (si existe)
        client.StopPlayback();

        float[] qRad = GetSlidersAsRad();
        client.RequestPlanQ(qRad);
    }

    // Botón RESET
    public void ResetPoseImmediate()
    {
        if (client != null) client.StopPlayback();

        if (!homeCaptured)
            CaptureHomePoseFromRobot();

        SetSlidersFromRad(homeRad);
        UpdateLabels();

        if (applier != null)
            applier.Apply(homeRad);
    }

    public void UpdateLabels()
    {
        if (q1Text) q1Text.text = $"Q1: {q1.value:0.0}°";
        if (q2Text) q2Text.text = $"Q2: {q2.value:0.0}°";
        if (q3Text) q3Text.text = $"Q3: {q3.value:0.0}°";
        if (q4Text) q4Text.text = $"Q4: {q4.value:0.0}°";
        if (q5Text) q5Text.text = $"Q5: {q5.value:0.0}°";
        if (q6Text) q6Text.text = $"Q6: {q6.value:0.0}°";
    }

    void CaptureHomePoseFromRobot()
    {
        // Si no asignaste j1..j6, no podemos leer pose real
        if (j1 == null || j2 == null || j3 == null || j4 == null || j5 == null || j6 == null)
        {
            Debug.LogWarning("[UI] j1..j6 are not assigned. Home pose will be zeros.");
            for (int i = 0; i < 6; i++) homeRad[i] = 0f;
            homeCaptured = true;
            SetSlidersFromRad(homeRad);
            return;
        }

        homeRad[0] = ReadJointRad(j1);
        homeRad[1] = ReadJointRad(j2);
        homeRad[2] = ReadJointRad(j3);
        homeRad[3] = ReadJointRad(j4);
        homeRad[4] = ReadJointRad(j5);
        homeRad[5] = ReadJointRad(j6);

        homeCaptured = true;
        SetSlidersFromRad(homeRad);

        Debug.Log($"[UI] Home captured (deg): " +
                  $"{homeRad[0] * Mathf.Rad2Deg:0.0}, {homeRad[1] * Mathf.Rad2Deg:0.0}, {homeRad[2] * Mathf.Rad2Deg:0.0}, " +
                  $"{homeRad[3] * Mathf.Rad2Deg:0.0}, {homeRad[4] * Mathf.Rad2Deg:0.0}, {homeRad[5] * Mathf.Rad2Deg:0.0}");
    }

    float ReadJointRad(UrdfJointRevolute j)
    {
        if (j == null) return 0f;

        // Mejor opción si URDF usa articulations:
        var ab = j.GetComponent<ArticulationBody>();
        if (ab != null)
            return ab.jointPosition[0]; // rad

        // Fallback (puede no coincidir en UR5e):
        float z = j.transform.localEulerAngles.z;
        if (z > 180f) z -= 360f;
        return z * Mathf.Deg2Rad;
    }

    float[] GetSlidersAsRad()
    {
        return new float[6]
        {
            (q1 ? q1.value : 0f) * Mathf.Deg2Rad,
            (q2 ? q2.value : 0f) * Mathf.Deg2Rad,
            (q3 ? q3.value : 0f) * Mathf.Deg2Rad,
            (q4 ? q4.value : 0f) * Mathf.Deg2Rad,
            (q5 ? q5.value : 0f) * Mathf.Deg2Rad,
            (q6 ? q6.value : 0f) * Mathf.Deg2Rad
        };
    }

    void SetSlidersFromRad(float[] qRad)
    {
        if (qRad == null || qRad.Length < 6) return;

        if (q1) q1.SetValueWithoutNotify(qRad[0] * Mathf.Rad2Deg);
        if (q2) q2.SetValueWithoutNotify(qRad[1] * Mathf.Rad2Deg);
        if (q3) q3.SetValueWithoutNotify(qRad[2] * Mathf.Rad2Deg);
        if (q4) q4.SetValueWithoutNotify(qRad[3] * Mathf.Rad2Deg);
        if (q5) q5.SetValueWithoutNotify(qRad[4] * Mathf.Rad2Deg);
        if (q6) q6.SetValueWithoutNotify(qRad[5] * Mathf.Rad2Deg);
    }
}
