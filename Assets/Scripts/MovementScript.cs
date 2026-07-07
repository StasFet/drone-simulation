using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class MovementScript : MonoBehaviour {

	public Rigidbody rb;
	public Transform[] motors;
	private GUIStyle styleGreenText = new GUIStyle();
	private GUIStyle styleGrayBG = new GUIStyle();
	public Font uiFont;
	public int uiFontSize = 22;
	private int numMotors = 4;
	public float maxSPIncrease = 9f;    // setpoint ramping for altitude
	public float[] motorThrusts = new float[4];
	private float maxThrustPerMotor = 8f;

	// state control and tracking
	public float altitude = 1f;
	public Quaternion currentRotation = new Quaternion();
	public Quaternion targetRotation = new Quaternion();
	public float targetAltitude = 1f;

	// --- PID STUFF
	public float[] rollPitchInnerPIDCoeffs;
	public float[] rollPitchOuterPIDCoeffs;
	PIDFController altitudeController = new PIDFController(20f, 0f, 8f);
	PIDFController thrustController = new PIDFController(kp: 0.3f, kf: 0.4596f);
	PIDFController rollOuterController = new PIDFController(10f, 0f, 2f);
    PIDFController pitchOuterController = new PIDFController(10f, 0f, 2f);
    PIDFController rollInnerController = new PIDFController(kp: 0.04f);
    PIDFController pitchInnerController = new PIDFController(kp: 0.04f);
	private Vector3 errVec = new Vector3();
	public float[] altPIDCoeffs = new float[4];
	public float[] thrustPIDCoeffs = new float[2];

	// tracking for GUI
	private float rollPIDCalc = 0f;
	private float pitchPIDCalc = 0f;
    private float targetThrustAltitude = 0f;            // the thrust needed to control altitude
    private float targetAngVelRoll = 0f;                // the angular velocity (rad) needed to control roll
    private float targetAngVelPitch = 0f;               // the angular velocity (rad) needed to control pitch
    private float currentTargetAltitude = 1f;           // ramped setpoint

    private void Start() {
		Texture2D background = new Texture2D(1, 1);
		background.SetPixel(0, 0, new Color(0.1f, 0.1f, 0.1f, 0.6f));
		background.Apply();
		styleGrayBG.normal.background = background;
		styleGrayBG.padding = new RectOffset(10, 10, 10, 10);

		styleGreenText.normal.textColor = Color.green;
		styleGreenText.font = uiFont;
		styleGreenText.fontSize = uiFontSize;
	}

	// lower frequency
	void Update() {
		altitudeController.Kp = altPIDCoeffs[0];
        altitudeController.Ki = altPIDCoeffs[1];
        altitudeController.Kd = altPIDCoeffs[2];

		thrustController.Kp = thrustPIDCoeffs[0];
        thrustController.Kf = thrustPIDCoeffs[1];

        rollInnerController.Kp = rollPitchInnerPIDCoeffs[0];
        pitchInnerController.Kp = rollPitchInnerPIDCoeffs[0];

        rollOuterController.Kp = rollPitchOuterPIDCoeffs[0];
		rollOuterController.Ki = rollPitchOuterPIDCoeffs[1];
		rollOuterController.Kd = rollPitchOuterPIDCoeffs[2];
        pitchOuterController.Kp = rollPitchOuterPIDCoeffs[0];
        pitchOuterController.Ki = rollPitchOuterPIDCoeffs[1];
        pitchOuterController.Kd = rollPitchOuterPIDCoeffs[2];

        // make target rotation from input
        targetRotation = Quaternion.Euler(0f, 0f, 0f);
		if (Keyboard.current.wKey.isPressed) targetRotation = Quaternion.Euler(25f, 0f, 0f) ;
		else if (Keyboard.current.sKey.isPressed) targetRotation = Quaternion.Euler(-25f, 0f, 0f);
		if (Keyboard.current.aKey.isPressed) targetRotation = Quaternion.Euler(0f, 0f, 25f) * targetRotation;
		else if (Keyboard.current.dKey.isPressed) targetRotation = Quaternion.Euler(0f, 0f, -25f) * targetRotation;

		// control altitude
		if (Keyboard.current.spaceKey.isPressed) targetAltitude += 0.05f;
		if (Keyboard.current.shiftKey.isPressed) targetAltitude -= 0.05f;
	}

	// higher frequency
	void FixedUpdate() {
        // state updates
        altitude = rb.position.y;
        currentRotation = rb.rotation;

        // outer loops
        targetThrustAltitude = altitudeController.Update(currentTargetAltitude, altitude, Time.deltaTime);  // outer controller 
        errVec = makeErrorVec(targetRotation, currentRotation);
        targetAngVelRoll = rollOuterController.Update(errVec.z, Time.deltaTime);
        targetAngVelPitch = pitchOuterController.Update(errVec.x, Time.deltaTime);

        // setpoint ramp
        currentTargetAltitude = rampSetpoint(currentTargetAltitude, targetAltitude, maxSPIncrease, Time.deltaTime);

		// inner loops
        float altitudePIDCalc = thrustController.Update(targetThrustAltitude, motorThrusts.Sum() / 4f, Time.deltaTime);
		rollPIDCalc = rollInnerController.Update(targetAngVelRoll, rb.angularVelocity.z, Time.deltaTime);
		pitchPIDCalc = pitchInnerController.Update(targetAngVelPitch, rb.angularVelocity.x, Time.deltaTime);
		motorThrusts[0] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc + rollPIDCalc - pitchPIDCalc));
        motorThrusts[1] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc - rollPIDCalc - pitchPIDCalc));
        motorThrusts[2] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc + rollPIDCalc + pitchPIDCalc));
        motorThrusts[3] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc - rollPIDCalc + pitchPIDCalc));

		for (int i = 0; i < numMotors; i++) {
			setMotorPower(motorThrusts[i], i);
		}
	}

	void OnGUI() {
		GUI.BeginGroup(new Rect(10f, 10f, 350f, 140f), styleGrayBG);
		GUI.Label(new Rect(10, 10, 300, 25), "Target Thrust: " + targetThrustAltitude, styleGreenText);
		for (int i = 0; i < numMotors; i++) {
            GUI.Label(new Rect(10, 10 + (i+1) * 25, 300, 25), "Motor " + i + " thrust: " + motorThrusts[i], styleGreenText);
        }
		GUI.EndGroup();

		GUI.BeginGroup(new Rect(400, 10, 350, 90), styleGrayBG);
        GUI.Label(new Rect(10, 10, 200, 20), "Target Altitude: " + targetAltitude, styleGreenText);
		GUI.Label(new Rect(10, 35, 200, 20), "Smoothed Target Altitude: " + currentTargetAltitude, styleGreenText);
		GUI.Label(new Rect(10, 60, 200, 20), "Current Altitude: " + altitude, styleGreenText);
		GUI.EndGroup();

        GUI.BeginGroup(new Rect(10, 200, 350, 90), styleGrayBG);
        GUI.Label(new Rect(10, 10, 200, 20), "Roll: " + currentRotation.eulerAngles.z, styleGreenText);
		GUI.Label(new Rect(10, 35, 200, 20), "Pitch: " + currentRotation.eulerAngles.x, styleGreenText);
		GUI.Label(new Rect(10, 60, 200, 20), "Yaw: " + currentRotation.eulerAngles.y, styleGreenText);
        GUI.EndGroup();

        GUI.BeginGroup(new Rect(10, 300, 350, 90), styleGrayBG);
        GUI.Label(new Rect(10, 10, 200, 20), "RollErr: " + errVec.z, styleGreenText);
		GUI.Label(new Rect(10, 35, 200, 20), "PitchErr: " + errVec.x, styleGreenText);
		GUI.Label(new Rect(10, 60, 200, 20), "YawErr: " + errVec.y, styleGreenText);
        GUI.EndGroup();

        GUI.BeginGroup(new Rect(10, 400, 350, 70), styleGrayBG);
        GUI.Label(new Rect(10, 10, 200, 20), "Roll Target AngVel: " + targetAngVelRoll, styleGreenText);
        GUI.Label(new Rect(10, 35, 200, 20), "Pitch Target AngVel: " + targetAngVelPitch, styleGreenText);
        GUI.EndGroup();

        GUI.BeginGroup(new Rect(10, 490, 350, 70), styleGrayBG);
        GUI.Label(new Rect(10, 10, 200, 20), "Roll Thrust: " + rollPIDCalc, styleGreenText);
        GUI.Label(new Rect(10, 35, 200, 20), "Pitch Thrust: " + pitchPIDCalc, styleGreenText);
        GUI.EndGroup();
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

	Vector3 makeErrorVec(Quaternion target, Quaternion current) {
		if (Quaternion.Dot(target, current) < 0f) target = Negate(target); // ensure it takes the shortest path
		Quaternion err = target * Quaternion.Inverse(current);
		err.ToAngleAxis(out float angle, out Vector3 axis);
		if (angle > 180f) angle -= 360f;
		return axis.normalized * angle * Mathf.Deg2Rad;     // stores 
	}

	private Quaternion Negate(Quaternion q) {
		return new Quaternion(-q.x, -q.y, -q.z, -q.w);
	}
}
