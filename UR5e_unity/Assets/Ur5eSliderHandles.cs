using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Unity.Robotics.UrdfImporter;

public class Ur5eSliderHandles : MonoBehaviour
{
    [Header("Six sliders (values in RADIANS)")]
    public Slider q1, q2, q3, q4, q5, q6;

    [Header("Texts (TMP) next to sliders (optional)")]
    public TMP_Text q1Text, q2Text, q3Text, q4Text, q5Text, q6Text;

    [Header("Client that requests PMP trajectory")]
    public Ur5eTrajectoryClientQ client;

    [Header("Apply joints in Unity")]
    public ApplyJointAngles6 applier;

    [Header("URDF joints (assign the Revolute joints here!)")]
    public UrdfJointRevolute j1, j2, j3, j4, j5, j6;

    [Header("Behavior")]
    public bool previewInUnity = false;
    public bool syncSlidersOnStart = true;

    [Tooltip("How many FixedUpdate steps to wait before syncing (URDF/articulations often need time).")]
    public int fixedUpdatesToWait = 20;

    [Header("Slider limits in radians")]
    public float minRad = -3.1416f;
    public float maxRad = 3.1416f;

    [Header("Optional snapping (radians)")]
    [Tooltip("0 = no snapping. Example: 0.01 rad is fine, 0.05 rad is coarse.")]
    public float stepRad = 0.01f;

    [Header("Debug")]
    public bool debugOnStart = false;

    private float[] homeRad = new float[6];
    private bool homeCaptured = false;

    IEnumerator Start()
    {
        yield return null;

        for (int i = 0; i < Mathf.Max(0, fixedUpdatesToWait); i++)
            yield return new WaitForFixedUpdate();

        SetupSlider(q1); SetupSlider(q2); SetupSlider(q3);
        SetupSlider(q4); SetupSlider(q5); SetupSlider(q6);

        if (syncSlidersOnStart)
            SyncSlidersToRobotPose();

        SnapAndClampAllSliders();
        UpdateLabels();

        if (debugOnStart)
            DumpAllJoints();
    }

    void SetupSlider(Slider s)
    {
        if (!s) return;
        s.minValue = minRad;
        s.maxValue = maxRad;
        s.wholeNumbers = false;
    }

    float Snap(float x)
    {
        if (stepRad <= 0f) return x;
        return Mathf.Round(x / stepRad) * stepRad;
    }

    void SnapClampOne(Slider s)
    {
        if (!s) return;
        float v = Mathf.Clamp(s.value, minRad, maxRad);
        v = Snap(v);
        s.SetValueWithoutNotify(v);
    }

    void SnapAndClampAllSliders()
    {
        SnapClampOne(q1); SnapClampOne(q2); SnapClampOne(q3);
        SnapClampOne(q4); SnapClampOne(q5); SnapClampOne(q6);
    }

  
    void SyncSlidersToRobotPose()
    {
        if (j1 != null && j2 != null && j3 != null && j4 != null && j5 != null && j6 != null)
        {
            float[] qRad = new float[6];
            qRad[0] = ReadJointRadRobust(j1, "j1");
            qRad[1] = ReadJointRadRobust(j2, "j2");
            qRad[2] = ReadJointRadRobust(j3, "j3");
            qRad[3] = ReadJointRadRobust(j4, "j4");
            qRad[4] = ReadJointRadRobust(j5, "j5");
            qRad[5] = ReadJointRadRobust(j6, "j6");

            SetSlidersFromRad(qRad);
            return;
        }

        if (applier != null && applier.LastQRad != null && applier.LastQRad.Length >= 6)
        {
            SetSlidersFromRad(applier.LastQRad);
            return;
        }

        Debug.LogWarning("[UI] No pude sincronizar: asigna j1..j6 (UrdfJointRevolute) o applier.LastQRad.");
    }

    float ReadJointRadRobust(UrdfJointRevolute j, string tag)
    {
        if (j == null) return 0f;

 
        ArticulationBody ab =
            j.GetComponent<ArticulationBody>() ??
            j.GetComponentInParent<ArticulationBody>() ??
            j.GetComponentInChildren<ArticulationBody>();

        if (ab != null)
        {

            if (ab.jointPosition.dofCount > 0)
            {
                float rad = ab.jointPosition[0];
                return rad;
            }


            float deg = ab.xDrive.target;
            return deg * Mathf.Deg2Rad;
        }


        float z = j.transform.localEulerAngles.z;
        if (z > 180f) z -= 360f;
        return z * Mathf.Deg2Rad;
    }

    void DumpAllJoints()
    {
        Debug.Log("=== [UI] Dump joints at Start ===");

        DumpOne(j1, "j1");
        DumpOne(j2, "j2");
        DumpOne(j3, "j3");
        DumpOne(j4, "j4");
        DumpOne(j5, "j5");
        DumpOne(j6, "j6");

        Debug.Log("=== [UI] End dump ===");
    }

    void DumpOne(UrdfJointRevolute j, string name)
    {
        if (j == null)
        {
            Debug.LogWarning($"{name}: NOT ASSIGNED (null)");
            return;
        }

        var ab =
            j.GetComponent<ArticulationBody>() ??
            j.GetComponentInParent<ArticulationBody>() ??
            j.GetComponentInChildren<ArticulationBody>();

        if (ab == null)
        {
            Debug.LogWarning($"{name}: no ArticulationBody found on joint/parent/children. GO={j.gameObject.name}");
            return;
        }

        float radPos = (ab.jointPosition.dofCount > 0) ? ab.jointPosition[0] : 0f;
        float degTarget = ab.xDrive.target;

        Debug.Log($"{name}: GO={j.gameObject.name} | abGO={ab.gameObject.name} | jointPos={radPos:0.000} rad ({radPos * Mathf.Rad2Deg:0.0}°) | xDrive.target={degTarget:0.0}°");
    }


    public void OnSliderChanged()
    {
        SnapAndClampAllSliders();
        UpdateLabels();

        if (previewInUnity && applier != null)
            applier.Apply(GetSlidersAsRad());
    }


    public void SendTarget()
    {
        if (client == null)
        {
            Debug.LogError("Ur5eSliderHandles: client is null");
            return;
        }

        client.StopPlayback();
        client.RequestPlanQ(GetSlidersAsRad());
    }

    public void ResetPoseImmediate()
    {
        if (client != null) client.StopPlayback();

        if (!homeCaptured)
            CaptureHomePoseFromRobot();

        SetSlidersFromRad(homeRad);

        if (applier != null)
            applier.Apply(homeRad);
    }

    void CaptureHomePoseFromRobot()
    {
        if (j1 == null || j2 == null || j3 == null || j4 == null || j5 == null || j6 == null)
        {
            Debug.LogWarning("[UI] j1..j6 are not assigned. Home pose will be zeros.");
            for (int i = 0; i < 6; i++) homeRad[i] = 0f;
            homeCaptured = true;
            return;
        }

        homeRad[0] = ReadJointRadRobust(j1, "j1");
        homeRad[1] = ReadJointRadRobust(j2, "j2");
        homeRad[2] = ReadJointRadRobust(j3, "j3");
        homeRad[3] = ReadJointRadRobust(j4, "j4");
        homeRad[4] = ReadJointRadRobust(j5, "j5");
        homeRad[5] = ReadJointRadRobust(j6, "j6");

        homeCaptured = true;
    }

    float[] GetSlidersAsRad()
    {
        return new float[6]
        {
            (q1 ? q1.value : 0f),
            (q2 ? q2.value : 0f),
            (q3 ? q3.value : 0f),
            (q4 ? q4.value : 0f),
            (q5 ? q5.value : 0f),
            (q6 ? q6.value : 0f)
        };
    }


    void SetSlidersFromRad(float[] qRad)
    {
        if (qRad == null || qRad.Length < 6) return;

        if (q1) q1.SetValueWithoutNotify(Mathf.Clamp(qRad[0], minRad, maxRad));
        if (q2) q2.SetValueWithoutNotify(Mathf.Clamp(qRad[1], minRad, maxRad));
        if (q3) q3.SetValueWithoutNotify(Mathf.Clamp(qRad[2], minRad, maxRad));
        if (q4) q4.SetValueWithoutNotify(Mathf.Clamp(qRad[3], minRad, maxRad));
        if (q5) q5.SetValueWithoutNotify(Mathf.Clamp(qRad[4], minRad, maxRad));
        if (q6) q6.SetValueWithoutNotify(Mathf.Clamp(qRad[5], minRad, maxRad));

        SnapAndClampAllSliders();
        UpdateLabels();
    }

    // ======= LABELS =======
    public void UpdateLabels()
    {
        if (q1Text) q1Text.text = $"Q1: {q1.value:0.00} rad ({q1.value * Mathf.Rad2Deg:0.0}°)";
        if (q2Text) q2Text.text = $"Q2: {q2.value:0.00} rad ({q2.value * Mathf.Rad2Deg:0.0}°)";
        if (q3Text) q3Text.text = $"Q3: {q3.value:0.00} rad ({q3.value * Mathf.Rad2Deg:0.0}°)";
        if (q4Text) q4Text.text = $"Q4: {q4.value:0.00} rad ({q4.value * Mathf.Rad2Deg:0.0}°)";
        if (q5Text) q5Text.text = $"Q5: {q5.value:0.00} rad ({q5.value * Mathf.Rad2Deg:0.0}°)";
        if (q6Text) q6Text.text = $"Q6: {q6.value:0.00} rad ({q6.value * Mathf.Rad2Deg:0.0}°)";
    }
}
