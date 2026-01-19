using UnityEngine;

public class KeyboardJogSliders : MonoBehaviour
{
    public Ur5eSliderHandles ui;     // UI handler for robot joint sliders
    public float stepDeg = 2f;      // Step size in degrees per key press
    public bool enableJog = true;   // Enable/disable keyboard jogging

    float timer = 0f;               // Limits sending frequency to backend

    void Update()
    {
        if (!enableJog || ui == null) return;

        timer -= Time.deltaTime;
        bool changed = false;

        // Joint 1
        if (Input.GetKey(KeyCode.Q)) { ui.q1.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.A)) { ui.q1.value -= stepDeg; changed = true; }

        // Joint 2
        if (Input.GetKey(KeyCode.W)) { ui.q2.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.S)) { ui.q2.value -= stepDeg; changed = true; }

        // Joint 3
        if (Input.GetKey(KeyCode.E)) { ui.q3.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.D)) { ui.q3.value -= stepDeg; changed = true; }

        // Joint 4
        if (Input.GetKey(KeyCode.R)) { ui.q4.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.F)) { ui.q4.value -= stepDeg; changed = true; }

        // Joint 5
        if (Input.GetKey(KeyCode.T)) { ui.q5.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.G)) { ui.q5.value -= stepDeg; changed = true; }

        // Joint 6
        if (Input.GetKey(KeyCode.Y)) { ui.q6.value += stepDeg; changed = true; }
        if (Input.GetKey(KeyCode.H)) { ui.q6.value -= stepDeg; changed = true; }

        if (changed)
        {
            ui.UpdateLabels();
            if (timer <= 0f)
                ui.SendTarget();
        }
    }
}


