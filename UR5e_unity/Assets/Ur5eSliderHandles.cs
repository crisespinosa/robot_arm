using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.UrdfImporter;

public class Ur5eSliderHandles : MonoBehaviour
{
    [Header("Six sliders (values in degrees)")]
    public Slider q1, q2, q3, q4, q5, q6; // UI sliders store values in degrees

    [Header("Texts (TMP) next to sliders (optional)")]
    public TMP_Text q1Text, q2Text, q3Text, q4Text, q5Text, q6Text; // optional labels near sliders

    [Header("Client that requests PMP trajectory")]
    public Ur5eTrajectoryClientQ client; // sends q_target (rad) to backend and plays trajectory

    [Header("Apply joints in Unity")]
    public ApplyJointAngles6 applier; // applies joint angles (rad) to URDF joints in Unity

    [Header("URDF joints (for reading initial pose)")]
    public UrdfJointRevolute j1, j2, j3, j4, j5, j6; // used only to read initial/home pose from robot

    [Header("Behavior")]
    [Tooltip("If true: robot moves immediately while you drag sliders (smooth preview). If false: robot moves ONLY after pressing Send.")]
    public bool previewInUnity = false; // preview movement while dragging sliders

    [Tooltip("If true: set sliders to current robot pose at start (recommended).")]
    public bool syncSlidersOnStart = true; // (not used below, but intended as a start behavior flag)

    // Home pose (rad)
    private float[] homeRad = new float[6]; // cached home pose in radians
    private bool homeCaptured = false;      // whether home pose was captured

    IEnumerator Start()
    {
        // Wait a couple of frames so URDF/ArticulationBodies finish initialization
        yield return null;
        yield return null;

        // Initialize sliders from the last applied pose (if available)
        if (applier != null && applier.LastQRad != null && applier.LastQRad.Length >= 6)
        {
            SetSlidersFromRad(applier.LastQRad);
            UpdateLabels();
        }
    }

    // Hook this method to OnValueChanged of each slider
    public void OnSliderChanged()
    {
        UpdateLabels();

        // Optional smooth preview: apply immediately while dragging
        if (previewInUnity && applier != null)
            applier.Apply(GetSlidersAsRad());
    }

    // SEND button: ask backend to plan and play a trajectory to current slider target
    public void SendTarget()
    {
        if (client == null)
        {
            Debug.LogError("Ur5eSliderHandles: client is null");
            return;
        }

        client.StopPlayback();             // stop any previous trajectory
        float[] qRad = GetSlidersAsRad();  // convert slider degrees -> radians
        client.RequestPlanQ(qRad);         // send to backend and start playback
    }

    // RESET button: stop playback, restore home pose immediately in Unity
    public void ResetPoseImmediate()
    {
        if (client != null) client.StopPlayback();

        if (!homeCaptured)
            CaptureHomePoseFromRobot(); // read home pose once and cache it

        SetSlidersFromRad(homeRad);
        UpdateLabels();

        if (applier != null)
            applier.Apply(homeRad);
    }

    // Updates the text labels next to sliders
    public void UpdateLabels()
    {
        if (q1Text) q1Text.text = $"Q1: {q1.value:0.0}°";
        if (q2Text) q2Text.text = $"Q2: {q2.value:0.0}°";
        if (q3Text) q3Text.text = $"Q3: {q3.value:0.0}°";
        if (q4Text) q4Text.text = $"Q4: {q4.value:0.0}°";
        if (q5Text) q5Text.text = $"Q5: {q5.value:0.0}°";
        if (q6Text) q6Text.text = $"Q6: {q6.value:0.0}°";
    }

    // Captures a "home" pose from URDF joints (preferred) or falls back to zeros
    void CaptureHomePoseFromRobot()
    {
        // If joints are not assigned, we cannot read real pose -> use zeros
        if (j1 == null || j2 == null || j3 == null || j4 == null || j5 == null || j6 == null)
        {
            Debug.LogWarning("[UI] j1..j6 are not assigned. Home pose will be zeros.");
            for (int i = 0; i < 6; i++) homeRad[i] = 0f;
            homeCaptured = true;
            SetSlidersFromRad(homeRad);
            return;
        }

        // Read current joint positions (radians) as home
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

    // Reads joint position in radians (ArticulationBody preferred)
    float ReadJointRad(UrdfJointRevolute j)
    {
        if (j == null) return 0f;

        var ab = j.GetComponent<ArticulationBody>();
        if (ab != null)
            return ab.jointPosition[0]; // radians

        // Fallback: estimate from local Z rotation (may not match UR5e joint axis)
        float z = j.transform.localEulerAngles.z;
        if (z > 180f) z -= 360f;
        return z * Mathf.Deg2Rad;
    }

    // Converts current slider values (degrees) into a 6-element radians array
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

    // Sets sliders from radians without triggering OnValueChanged callbacks
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






