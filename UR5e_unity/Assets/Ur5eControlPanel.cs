using UnityEngine;

/// <summary>
/// Runtime UI panel for controlling the UR5e PPO Controller.
/// Creates sliders for joint targets and trajectory parameters,
/// plus Start/Stop buttons — no Inspector needed.
///
/// Usage:
///   1. Add this script to ANY GameObject in the scene
///   2. Drag the Ur5ePPOController reference into the "controller" field
///   3. Press Play — the panel appears in the top-left corner
/// </summary>
public class Ur5eControlPanel : MonoBehaviour
{
    [Header("Reference")]
    [Tooltip("Drag your Ur5ePPOController here")]
    public Ur5ePPOController controller;

    // ── Joint limits (UR5e, radians) ──
    // Joints 0,3,4,5: ±2π  |  Joints 1,2: -π to π
    private static readonly float[] jointMin = { -6.2832f, -3.1416f, -3.1416f, -6.2832f, -6.2832f, -6.2832f };
    private static readonly float[] jointMax = {  6.2832f,  3.1416f,  3.1416f,  6.2832f,  6.2832f,  6.2832f };

    private static readonly string[] jointNames =
        { "Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3" };

    // ── Layout constants ──
    private const float PANEL_X = 10f;
    private const float PANEL_Y = 10f;
    private const float PANEL_W = 340f;
    private const float ROW_H = 22f;
    private const float PAD = 6f;
    private const float LABEL_W = 80f;
    private const float VALUE_W = 55f;

    // ── Internal state ──
    private bool panelVisible = true;
    private Vector2 scrollPos;
    private GUIStyle headerStyle;
    private GUIStyle valueStyle;
    private GUIStyle sectionStyle;
    private bool stylesReady = false;

    // Trajectory param copies (so sliders work smoothly)
    private float trajT;
    private float trajDt;
    private int trajN;

    void Start()
    {
        if (controller == null)
            controller = FindObjectOfType<Ur5ePPOController>();

        if (controller != null)
        {
            trajT  = controller.T;
            trajDt = controller.dt;
            trajN  = controller.N;
        }
    }

    private void InitStyles()
    {
        headerStyle = new GUIStyle(GUI.skin.label)
        {
            fontStyle = FontStyle.Bold,
            fontSize  = 14,
            normal    = { textColor = Color.white }
        };

        sectionStyle = new GUIStyle(GUI.skin.label)
        {
            fontStyle = FontStyle.Bold,
            fontSize  = 11,
            normal    = { textColor = new Color(0.6f, 0.85f, 1f) }
        };

        valueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize  = 11,
            alignment = TextAnchor.MiddleRight,
            normal    = { textColor = new Color(1f, 1f, 0.6f) }
        };

        stylesReady = true;
    }

    void OnGUI()
    {
        if (!stylesReady) InitStyles();

        // Toggle button (always visible)
        if (GUI.Button(new Rect(PANEL_X, PANEL_Y, 30, 24), panelVisible ? "−" : "+"))
            panelVisible = !panelVisible;

        GUI.Label(new Rect(PANEL_X + 34, PANEL_Y + 2, 200, 22),
                  "UR5e Control Panel", headerStyle);

        if (!panelVisible || controller == null) return;

        // Calculate panel height
        float contentH = 420f;
        float panelH = Mathf.Min(contentH, Screen.height - PANEL_Y - 20);

        // Semi-transparent background
        GUI.color = new Color(0, 0, 0, 0.8f);
        GUI.DrawTexture(new Rect(PANEL_X, PANEL_Y + 28, PANEL_W, panelH),
                        Texture2D.whiteTexture);
        GUI.color = Color.white;

        GUILayout.BeginArea(new Rect(PANEL_X + PAD, PANEL_Y + 32,
                                      PANEL_W - PAD * 2, panelH - 8));

        scrollPos = GUILayout.BeginScrollView(scrollPos);

        // ═══════════════════════════════════════
        // TARGET JOINT ANGLES
        // ═══════════════════════════════════════
        GUILayout.Label("Target Joint Angles (rad)", sectionStyle);
        GUILayout.Space(2);

        for (int i = 0; i < 6; i++)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(jointNames[i], GUILayout.Width(LABEL_W));

            controller.qTarget[i] = GUILayout.HorizontalSlider(
                controller.qTarget[i], jointMin[i], jointMax[i]);

            GUILayout.Label(controller.qTarget[i].ToString("F2"),
                            valueStyle, GUILayout.Width(VALUE_W));
            GUILayout.EndHorizontal();
        }

        // Quick-reset target to zeros
        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Reset to Zero", GUILayout.Height(24)))
        {
            for (int i = 0; i < 6; i++)
                controller.qTarget[i] = 0f;
        }
        if (GUILayout.Button("Random Target", GUILayout.Height(24)))
        {
            for (int i = 0; i < 6; i++)
                controller.qTarget[i] = Random.Range(jointMin[i] * 0.3f,
                                                      jointMax[i] * 0.3f);
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(8);

        // ═══════════════════════════════════════
        // TRAJECTORY PARAMETERS
        // ═══════════════════════════════════════
        GUILayout.Label("Trajectory Parameters", sectionStyle);
        GUILayout.Space(2);

        // T (duration)
        GUILayout.BeginHorizontal();
        GUILayout.Label("T (s)", GUILayout.Width(LABEL_W));
        trajT = GUILayout.HorizontalSlider(trajT, 0.5f, 5.0f);
        GUILayout.Label(trajT.ToString("F2"), valueStyle, GUILayout.Width(VALUE_W));
        GUILayout.EndHorizontal();
        controller.T = Mathf.Round(trajT * 20f) / 20f; // snap to 0.05

        // dt
        GUILayout.BeginHorizontal();
        GUILayout.Label("dt (s)", GUILayout.Width(LABEL_W));
        trajDt = GUILayout.HorizontalSlider(trajDt, 0.005f, 0.05f);
        GUILayout.Label(trajDt.ToString("F3"), valueStyle, GUILayout.Width(VALUE_W));
        GUILayout.EndHorizontal();
        controller.dt = Mathf.Round(trajDt * 1000f) / 1000f; // snap to 0.001

        // N (horizon)
        GUILayout.BeginHorizontal();
        GUILayout.Label("N (horizon)", GUILayout.Width(LABEL_W));
        float nFloat = GUILayout.HorizontalSlider(trajN, 5, 50);
        trajN = Mathf.RoundToInt(nFloat);
        GUILayout.Label(trajN.ToString(), valueStyle, GUILayout.Width(VALUE_W));
        GUILayout.EndHorizontal();
        controller.N = trajN;

        // Mode display
        GUILayout.BeginHorizontal();
        GUILayout.Label("Mode", GUILayout.Width(LABEL_W));
        GUILayout.Label(controller.mode, GUILayout.Width(100));
        GUILayout.EndHorizontal();

        GUILayout.Space(8);

        // ═══════════════════════════════════════
        // EPISODE CONTROL
        // ═══════════════════════════════════════
        GUILayout.Label("Episode Control", sectionStyle);
        GUILayout.Space(2);

        GUILayout.BeginHorizontal();
        GUI.backgroundColor = new Color(0.2f, 0.8f, 0.3f);
        if (GUILayout.Button("▶  Start Episode", GUILayout.Height(32)))
            controller.StartEpisode();

        GUI.backgroundColor = new Color(0.9f, 0.3f, 0.2f);
        if (GUILayout.Button("■  Stop", GUILayout.Height(32)))
            controller.StopEpisode();

        GUI.backgroundColor = Color.white;
        GUILayout.EndHorizontal();

        GUILayout.Space(4);
        GUILayout.Label("Keyboard: P = Start  |  O = Stop",
                        new GUIStyle(GUI.skin.label) { fontSize = 10,
                            normal = { textColor = Color.gray } });

        GUILayout.Space(8);

        // ═══════════════════════════════════════
        // LIVE STATUS
        // ═══════════════════════════════════════
        GUILayout.Label("Live Status", sectionStyle);
        GUILayout.Space(2);

        DrawStatusRow("Episode Active", controller.GetEpisodeActive() ? "YES" : "no",
                       controller.GetEpisodeActive() ? Color.green : Color.gray);
        DrawStatusRow("Step", controller.GetStepCount().ToString(), Color.white);
        DrawStatusRow("Reward", controller.GetTotalReward().ToString("F2"), Color.white);
        DrawStatusRow("eq_rms", controller.GetLastEqRms().ToString("F4"),
                       controller.GetLastEqRms() < 0.01f ? Color.green : Color.yellow);
        DrawStatusRow("Success", controller.GetLastSuccess() ? "YES" : "no",
                       controller.GetLastSuccess() ? Color.green : Color.gray);

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    private void DrawStatusRow(string label, string value, Color color)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, GUILayout.Width(LABEL_W));
        var st = new GUIStyle(GUI.skin.label)
        {
            normal = { textColor = color },
            fontStyle = FontStyle.Bold
        };
        GUILayout.Label(value, st);
        GUILayout.EndHorizontal();
    }
}
