using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;
using Newtonsoft.Json;

public class Ur5eTrajectoryClientQ : MonoBehaviour
{
    [Header("Backend")]
    public string serverIP = "127.0.0.1";
    public int port = 8848;
    public string path = "/arm/plan_pmp_q";

    [Header("Trajectory params")]
    public float T = 1.0f;
    public float dt = 0.02f;

    [Header("Playback speed")]
    [Tooltip("1 = normal, 2 = 2x slower, 0.5 = 2x faster")]
    public float playbackScale = 1.0f;

    [Header("Applier")]
    public ApplyJointAngles6 applier;

    Coroutine playRoutine;

    // ---------- JSON ----------
    [System.Serializable]
    class PlanQRequest
    {
        public float[] q_target;
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

    IEnumerator PostPlanQ(float[] qTargetRad)
    {
        string url = BuildUrl();

        var bodyObj = new PlanQRequest
        {
            q_target = qTargetRad,
            T = T,
            dt = dt
        };

        string bodyJson = JsonConvert.SerializeObject(bodyObj);
        Debug.Log("[TrajectoryClientQ] POST " + url);
        Debug.Log("[TrajectoryClientQ] body=" + bodyJson);

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
        // (Opcional) log corto
        Debug.Log("[TrajectoryClientQ] got response, len=" + raw.Length);

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
        Debug.Log($"[TrajectoryClientQ] Playing trajectory ({n} points)");

        float wait = resp.dt * Mathf.Max(0.0001f, playbackScale);

        // ✅ Logs para probar si cambian
        var p0 = resp.trajectory[0];
        var pm = resp.trajectory[n / 2];
        var pL = resp.trajectory[n - 1];

        Debug.Log($"[TrajectoryClientQ] q(first) = {p0.q[0]:F3}, {p0.q[1]:F3}");
        Debug.Log($"[TrajectoryClientQ] q(mid)   = {pm.q[0]:F3}, {pm.q[1]:F3}");
        Debug.Log($"[TrajectoryClientQ] q(last)  = {pL.q[0]:F3}, {pL.q[1]:F3}");
        Debug.Log($"wait = {resp.dt * playbackScale}");

        for (int i = 0; i < n; i++)
        {
            var p = resp.trajectory[i];
            if (p?.q == null || p.q.Length < 6) continue;

            applier.Apply(p.q);

            yield return new WaitForSeconds(wait);
        }
        // lock final pose a few times to let physics settle
        var last = resp.trajectory[resp.trajectory.Count - 1];
        for (int k = 0; k < 10; k++)
        {
            applier.Apply(last.q);
            yield return null; // one frame
        }

        playRoutine = null;
        Debug.Log("[TrajectoryClientQ] Playback finished");
    }
}
