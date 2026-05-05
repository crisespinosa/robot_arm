using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;
using Unity.Robotics.UrdfImporter;

public class Ur5eTrajectoryClientQ : MonoBehaviour
{
    [Header("Backend")]
    public string serverIP = "127.0.0.1";
    public int port = 8848;
    public string path = "/arm/plan_minjerk_q";

    [Header("Trajectory params")]
    public float T = 1.0f;
    public float dt = 0.02f;

    [Header("Playback speed")]
    [Tooltip("1 = normal, 2 = 2x slower, 0.5 = 2x faster")]
    public float playbackScale = 1.0f;

    [Header("Applier (must have q1..q6 assigned)")]
    public ApplyJointAngles6 applier;



    Coroutine playRoutine;

    // Guardamos para pasarle al logger
    float[] lastQTarget;
    float[] lastQStart;

    // ---------- JSON ----------
    [System.Serializable]
    class PlanQRequest
    {
        public float[] q_target;
        public float[] q_start;   
        public float T;
        public float dt;
    }

    [System.Serializable]
    class TrajPoint
    {
        public float t;
        public float[] q;
    }

    [System.Serializable]
    class TrajResponse
    {
        public float dt;
        public string unit;
        public List<TrajPoint> trajectory;

        // opcional: debug fields si el backend los manda
        public float[] q_start_used;
        public float[] q_target_used;
    }

    string BuildUrl() => $"http://{serverIP}:{port}{path}";

    public void RequestPlanQ(float[] qTargetRad)
    {
        if (applier == null)
        {
            Debug.LogError("[TrajectoryClientQ] applier is NULL (assign ApplyJointAngles6 in Inspector).");
            return;
        }
        if (qTargetRad == null || qTargetRad.Length < 6)
        {
            Debug.LogError("[TrajectoryClientQ] qTargetRad must be length 6.");
            return;
        }

        StopPlayback();
        StartCoroutine(PostPlanQ(qTargetRad));
    }

    public void StopPlayback()
    {
        if (playRoutine != null)
        {
            StopCoroutine(playRoutine);
            playRoutine = null;
        }
    }

    float ReadJointRad(UrdfJointRevolute j)
    {
        if (j == null) return 0f;

        var ab = j.GetComponent<ArticulationBody>();
        if (ab != null)
            return ab.jointPosition[0]; 

      
        float z = j.transform.localEulerAngles.z;
        if (z > 180f) z -= 360f;
        return z * Mathf.Deg2Rad;
    }

    float[] GetUnityCurrentQRad()
    {
        
        if (applier != null && applier.q1 != null && applier.q2 != null && applier.q3 != null &&
            applier.q4 != null && applier.q5 != null && applier.q6 != null)
        {
            return new float[6]
            {
                ReadJointRad(applier.q1),
                ReadJointRad(applier.q2),
                ReadJointRad(applier.q3),
                ReadJointRad(applier.q4),
                ReadJointRad(applier.q5),
                ReadJointRad(applier.q6),
            };
        }

        
        if (applier != null && applier.LastQRad != null && applier.LastQRad.Length >= 6)
        {
            return new float[6]
            {
                applier.LastQRad[0], applier.LastQRad[1], applier.LastQRad[2],
                applier.LastQRad[3], applier.LastQRad[4], applier.LastQRad[5]
            };
        }

        return new float[6]; 
    }

    IEnumerator PostPlanQ(float[] qTargetRad)
    {
        string url = BuildUrl();

        float[] qStartRad = GetUnityCurrentQRad();

        // Guardar para el logger
        lastQTarget = (float[])qTargetRad.Clone();
        lastQStart = (float[])qStartRad.Clone();

        var bodyObj = new PlanQRequest
        {
            q_target = qTargetRad,
            q_start = qStartRad,
            T = T,
            dt = dt
        };

        string bodyJson = JsonConvert.SerializeObject(bodyObj);

        using var req = new UnityWebRequest(url, "POST");
        req.uploadHandler = new UploadHandlerRaw(System.Text.Encoding.UTF8.GetBytes(bodyJson));
        req.downloadHandler = new DownloadHandlerBuffer();
        req.SetRequestHeader("Content-Type", "application/json");
        req.SetRequestHeader("Accept", "application/json");

        yield return req.SendWebRequest();

        if (req.result != UnityWebRequest.Result.Success)
        {
            Debug.LogError("[TrajectoryClientQ] HTTP error: " + req.error);
            Debug.LogError("[TrajectoryClientQ] response: " + req.downloadHandler.text);
            yield break;
        }

        string raw = req.downloadHandler.text;

        TrajResponse resp;
        try
        {
            resp = JsonConvert.DeserializeObject<TrajResponse>(raw);
        }
        catch (System.Exception e)
        {
            Debug.LogError("[TrajectoryClientQ] JSON parse failed: " + e.Message);
            Debug.LogError(raw);
            yield break;
        }

        if (resp?.trajectory == null || resp.trajectory.Count == 0)
        {
            Debug.LogError("[TrajectoryClientQ] Empty trajectory.");
            yield break;
        }

        playRoutine = StartCoroutine(PlayTrajectory(resp));
    }

    IEnumerator PlayTrajectory(TrajResponse resp)
    {
        int n = resp.trajectory.Count;
        // Debug.Log($"[TrajectoryClientQ] Playing trajectory ({n} points)");

        for (int i = 0; i < n; i++)
        {
            var p = resp.trajectory[i];
            if (p?.q == null || p.q.Length < 6) continue;

            applier.Apply(p.q);

            if (i < n - 1)
            {
                float dtSeg = resp.trajectory[i + 1].t - p.t;
                yield return new WaitForSeconds(Mathf.Max(0.0001f, dtSeg * Mathf.Max(0.0001f, playbackScale)));
            }
        }

        var last = resp.trajectory[n - 1];
        for (int k = 0; k < 10; k++)
        {
            applier.Apply(last.q);
            yield return null;
        }

        playRoutine = null;
        // Debug.Log("[TrajectoryClientQ] Playback finished");

    }
}
