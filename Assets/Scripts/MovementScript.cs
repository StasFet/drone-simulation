using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class MovementScript : MonoBehaviour {

	public Rigidbody rb;
	public Transform[] motors;
	private GUIStyle styleRed = new GUIStyle();
	private GUIStyle styleGreen = new GUIStyle();
	private int numMotors = 4;
	public float maxSPIncrease = 9f;    // setpoint ramping for altitude
	public float[] motorThrusts = new float[4];
	private float maxThrustPerMotor = 8f;

	// state control and tracking
	public float altitude = 1f;
	public Quaternion currentRotation = new Quaternion();
	public Quaternion targetRotation = new Quaternion();
	public float targetAltitude = 1f;
	private float currentTargetAltitude = 1f;           // ramped setpoint

	// --- PID STUFF
	public float[] rollPitchInnerPIDCoeffs;
	public float[] rollPitchOuterPIDCoeffs;
	PIDFController altitudeController = new PIDFController(20f, 0f, 8f);
	PIDFController thrustController = new PIDFController(kp: 0.3f, kf: 0.4596f);
	PIDFController rollPitchOuterController = new PIDFController(1f, 0f, 0.1f);
	PIDFController rollPitchInnerController = new PIDFController(kp: 1f, kf: 0f);
	private float targetThrustAltitude = 0f;            // the thrust needed to control altitude
	public float targetAngVelRoll = 0f;                // the angular velocity (rad) needed to control roll
	public float targetAngVelPitch = 0f;               // the angular velocity (rad) needed to control pitch
	private Vector3 errVec = new Vector3();
	public float[] altPIDCoeffs = new float[4];
	public float[] thrustPIDCoeffs = new float[2];

	private void Start() {
		styleRed.normal.textColor = Color.red;
		styleGreen.normal.textColor = Color.green;
		styleRed.fontSize = 22;
		styleGreen.fontSize = 22;
	}

	// lower frequency
	void Update() {
		altitudeController.Kp = altPIDCoeffs[0];
        altitudeController.Ki = altPIDCoeffs[1];
        altitudeController.Kd = altPIDCoeffs[2];

		thrustController.Kp = thrustPIDCoeffs[0];
        thrustController.Kf = thrustPIDCoeffs[1];

        rollPitchInnerController.Kp = rollPitchInnerPIDCoeffs[0];
		rollPitchInnerController.Kf = rollPitchInnerPIDCoeffs[1];

		rollPitchOuterController.Kp = rollPitchOuterPIDCoeffs[0];
		rollPitchOuterController.Ki = rollPitchOuterPIDCoeffs[1];
		rollPitchOuterController.Kd = rollPitchOuterPIDCoeffs[2];

		// make target rotation from input
		targetRotation = Quaternion.Euler(0f, 0f, 0f);
		if (Keyboard.current.wKey.isPressed) targetRotation = Quaternion.Euler(0f, -25f, 0f) ;
		else if (Keyboard.current.sKey.isPressed) targetRotation = Quaternion.Euler(0f, 25f, 0f);
		if (Keyboard.current.aKey.isPressed) targetRotation = Quaternion.Euler(0f, 0f, -25f) * targetRotation;
		else if (Keyboard.current.dKey.isPressed) targetRotation = Quaternion.Euler(0f, 0f, 25f) * targetRotation;
	}

	// higher frequency
	void FixedUpdate() {
        // state updates
        altitude = rb.position.y;
        currentRotation = rb.rotation;

        // outer loops
        targetThrustAltitude = altitudeController.Update(currentTargetAltitude, altitude, Time.deltaTime);  // outer controller 
        errVec = makeErrorVec(targetRotation, currentRotation);
        targetAngVelRoll = rollPitchOuterController.Update(errVec.z, Time.deltaTime);
        targetAngVelPitch = rollPitchOuterController.Update(errVec.x, Time.deltaTime);

        // setpoint ramp
        currentTargetAltitude = rampSetpoint(currentTargetAltitude, targetAltitude, maxSPIncrease, Time.deltaTime);

		// inner loops
        float altitudePIDCalc = thrustController.Update(targetThrustAltitude, motorThrusts.Sum() / 4f, Time.deltaTime);
		float rollPIDCalc = 0; //rollPitchInnerController.Update(targetAngVelRoll, rb.angularVelocity.z, Time.deltaTime);
		float pitchPIDCalc = 0; //rollPitchInnerController.Update(targetAngVelPitch, rb.angularVelocity.x, Time.deltaTime);
		motorThrusts[0] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc + rollPIDCalc + pitchPIDCalc));
        motorThrusts[1] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc - rollPIDCalc + pitchPIDCalc));
        motorThrusts[2] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc + rollPIDCalc - pitchPIDCalc));
        motorThrusts[3] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc - rollPIDCalc - pitchPIDCalc));

		for (int i = 0; i < numMotors; i++) {
			setMotorPower(motorThrusts[i], i);
		}
	}

	void OnGUI() {
		GUI.Label(new Rect(10, 10, 400, 40), "Target Thrust: " + targetThrustAltitude, styleRed);
		for (int i = 0; i < numMotors; i++) {
            GUI.Label(new Rect(10, 10 + (i+1) * 20, 400, 40), "Motor " + i + " thrust: " + motorThrusts[i], styleRed);
        }

        GUI.Label(new Rect(1200, 650, 200, 20), "Target Altitude: " + targetAltitude, styleRed);
		GUI.Label(new Rect(1200, 675, 200, 20), "Smoothed Target Altitude: " + currentTargetAltitude, styleRed);
		GUI.Label(new Rect(1200, 700, 200, 20), "Current Altitude: " + altitude, styleRed);

		GUI.Label(new Rect(1200, 750, 200, 20), "Roll: " + currentRotation.eulerAngles.z, styleGreen);
		GUI.Label(new Rect(1200, 775, 200, 20), "Pitch: " + currentRotation.eulerAngles.x, styleGreen);
		GUI.Label(new Rect(1200, 800, 200, 20), "Yaw: " + currentRotation.eulerAngles.y, styleGreen);

		GUI.Label(new Rect(700, 400, 200, 20), "RollErr: " + errVec.z, styleGreen);
		GUI.Label(new Rect(700, 425, 200, 20), "PitchErr: " + errVec.x, styleGreen);
		GUI.Label(new Rect(700, 450, 200, 20), "YawErr: " + errVec.y, styleGreen);
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
		if (Quaternion.Dot(target, current) < 0f) target = Negate(current); // ensure it takes the shortest path
		Quaternion err = target * Quaternion.Inverse(current);
		err.ToAngleAxis(out float angle, out Vector3 axis);
		if (angle > 180f) angle -= 360f;
		return axis.normalized * angle * Mathf.Deg2Rad;     // stores 
	}

	private Quaternion Negate(Quaternion q) {
		return new Quaternion(-q.x, -q.y, -q.z, -q.w);
	}
}
