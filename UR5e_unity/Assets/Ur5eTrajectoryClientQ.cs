/*
Данный скрипт — клиент Unity для запроса траектории движения робота UR5e у backend-сервера
(например, планировщик PMP по суставным углам). Он отправляет целевую конфигурацию q_target
(6 углов в радианах) на HTTP endpoint, получает JSON с траекторией (список точек q(t)),
а затем воспроизводит траекторию в Unity, применяя каждый набор углов через ApplyJointAngles6.

Ключевые функции:
- RequestPlanQ(qTargetRad): отправляет запрос на планирование траектории к backend
- PostPlanQ(...): делает HTTP POST и парсит JSON-ответ
- PlayTrajectory(...): проигрывает траекторию с настраиваемой скоростью (playbackScale)
*/

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
    public float T = 1.0f;     // total trajectory duration (seconds)
    public float dt = 0.02f;   // timestep requested from backend (seconds)

    [Header("Playback speed")]
    [Tooltip("1 = normal, 2 = 2x slower, 0.5 = 2x faster")]
    public float playbackScale = 1.0f; // scales the wait time between points

    [Header("Applier")]
    public ApplyJointAngles6 applier;  // applies q (rad) to URDF joints

    Coroutine playRoutine; // currently running playback coroutine

    // ---- JSON DTOs (request/response structures) ----
    [System.Serializable]
    class PlanQRequest
    {
        public float[] q_target; // target joint angles (rad), length 6
        public float T;          // desired duration
        public float dt;         // desired timestep
    }

    [System.Serializable]
    class TrajPoint
    {
        public float t;   // time stamp (seconds)
        public float[] q; // joint angles at time t (rad), length 6
    }

    [System.Serializable]
    class TrajResponse
    {
        public float dt;                 // timestep actually used by backend
        public string unit;              // usually "rad"
        public List<TrajPoint> trajectory; // list of trajectory points
    }

    // Builds full URL like http://127.0.0.1:8848/arm/plan_pmp_q
    string BuildUrl() => $"http://{serverIP}:{port}{path}";

    // Public entry point: call this from UI / sliders when user sets a target configuration
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

        StopPlayback();                    // stop any previous trajectory playback
        StartCoroutine(PostPlanQ(qTargetRad)); // request a new planned trajectory
    }

    // Stops currently playing trajectory (if any)
    public void StopPlayback()
    {
        if (playRoutine != null)
        {
            StopCoroutine(playRoutine);
            playRoutine = null;
        }
    }

    // Coroutine that sends POST request to backend and starts playback on success
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

        playRoutine = StartCoroutine(PlayTrajectory(resp)); // start visual playback
    }

    // Plays the received trajectory in Unity by applying each q to the robot
    IEnumerator PlayTrajectory(TrajResponse resp)
    {
        int n = resp.trajectory.Count;
        Debug.Log($"[TrajectoryClientQ] Playing trajectory ({n} points)");

        // Wait time between points (scaled). >0 to avoid zero/negative waiting.
        float wait = resp.dt * Mathf.Max(0.0001f, playbackScale);

        // Minimal debug probes (first/mid/last) to verify that q values change
        var p0 = resp.trajectory[0];
        var pm = resp.trajectory[n / 2];
        var pL = resp.trajectory[n - 1];

        Debug.Log($"[TrajectoryClientQ] q(first) = {p0.q[0]:F3}, {p0.q[1]:F3}");
        Debug.Log($"[TrajectoryClientQ] q(mid)   = {pm.q[0]:F3}, {pm.q[1]:F3}");
        Debug.Log($"[TrajectoryClientQ] q(last)  = {pL.q[0]:F3}, {pL.q[1]:F3}");
        Debug.Log($"wait = {resp.dt * playbackScale}");

        // Main playback loop
        for (int i = 0; i < n; i++)
        {
            var p = resp.trajectory[i];
            if (p?.q == null || p.q.Length < 6) continue;

            applier.Apply(p.q); // ApplyJointAngles6 expects radians and converts internally
            yield return new WaitForSeconds(wait);
        }

        // Re-apply final pose a few frames to stabilize the final configuration
        var last = resp.trajectory[resp.trajectory.Count - 1];
        for (int k = 0; k < 10; k++)
        {
            applier.Apply(last.q);
            yield return null;
        }

        playRoutine = null;
        Debug.Log("[TrajectoryClientQ] Playback finished");
    }
}






