using UnityEngine;

public class MovementScript : MonoBehaviour {

    public Rigidbody rb;
    public Transform[] motors;
    private int numMotors = 4;
    public float maxSPIncrease = 9f;
    public float targetY = 1f;
    public float[] altPIDCoeffs = new float[3];
    public float[] thrustPIDCoeffs = new float[2];
    public float[] motorThrusts = new float[4];
    private float maxThrustPerMotor = 8f;
    private float currentTargetY = 1f;

    // --- PID STUFF
    PIDFController altitudeController = new PIDFController(20f, 0f, 8f);
    PIDFController thrustController = new PIDFController(kp: 0.3f, kf: 0.4596f);
    private float targetThrust = 0f;    // calculated by alt controller

    private GUIStyle style = new GUIStyle();

    private void Start() {      
        style.fontSize = 22;
        style.normal.textColor = Color.green;
    }

    // lower frequency
    void Update() {
        altitudeController.Kp = altPIDCoeffs[0];
        altitudeController.Ki = altPIDCoeffs[1];
        altitudeController.Kd = altPIDCoeffs[2];

        thrustController.Kp = thrustPIDCoeffs[0];
        thrustController.Kf = thrustPIDCoeffs[1];

        float verticalPos = rb.position.y;
        float pidCalc = thrustController.Update(targetThrust, motorThrusts[0], Time.deltaTime);
        for (int i = 0; i < numMotors; i++) {
            motorThrusts[i] = Mathf.Max(0f, Mathf.Min(pidCalc, 1f));
        }

        currentTargetY = rampSetpoint(currentTargetY, targetY, maxSPIncrease, Time.deltaTime);
    }

    // higher frequency
    void FixedUpdate() {
        for (int i = 0; i < numMotors; i++) {
            setMotorPower(motorThrusts[i], i);
        }

        targetThrust = altitudeController.Update(currentTargetY, rb.position.y, Time.deltaTime);
    }

    private void OnGUI() {
        GUI.Label(new Rect(10, 10, 400, 40), "Motor Thrust: " + motorThrusts[0], style);
        GUI.Label(new Rect(10, 35, 400, 40), "Target Thrust: " + targetThrust, style);

        GUI.Label(new Rect(1200, 650, 200, 20), "Target Altitude: " + targetY, style);
        GUI.Label(new Rect(1200, 675, 200, 20), "Smoothed Target Altitude: " + currentTargetY, style);
        GUI.Label(new Rect(1200, 700, 200, 20), "Current Altitude: " + rb.position.y, style);
    }

    void setMotorPower(float power, int index) {
        Vector3 force = motors[index].up * Mathf.Max(0f, Mathf.Min(1f, power)) * maxThrustPerMotor;
        rb.AddForceAtPosition(force, motors[index].position);
    }

    float rampSetpoint(float currentSP, float targetSP, float maxRate, float deltaTime) {
        float maxStep = maxRate * deltaTime;
        float delta = targetSP - currentSP;
        delta = Mathf.Max(-maxStep, Mathf.Min(maxStep, delta));
        return currentSP + delta;
    }
}
