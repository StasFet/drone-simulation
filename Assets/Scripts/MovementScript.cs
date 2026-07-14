using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

public class MovementScript : MonoBehaviour {

	// TODO: 
	// - implement axis flipping for x1 = x2

	public Rigidbody rb;
	public Transform[] motors;
    public int[] motorSpinDirection = { -1, 1, 1, -1 };
	private GUIStyle styleGreenText = new GUIStyle();
	private GUIStyle styleGrayBG = new GUIStyle();
	public Font uiFont;
	public int uiFontSize = 22;
	private int numMotors = 4;
    public bool showFullTelemetry = true;
    public bool showLocationTelemetry = true;
	public float maxSPIncreaseVel = 5f; // max SP increase rate for velocity
    public float maxSPIcreaseLoc = 10f; // same for location
	private float maxThrustPerMotor = 8f;
	public bool velocityControlMode = false;    // this is drone-centric
	public bool rotationControlMode = false;    // also drone-centric
	public bool positionControlMode = true;     // absolute
	public float velCtrlModeMaxSpeed = 8f;
	public float ctrlModeAscentSpeed = 0.03f;
    public float rotCtrlModeRotAmt = 25f;
    public float thrustToOmegaCoeff = 0.1f;
    public float omegaToReactionCoeff = 0.0015f;
    public float yawSpeed = 20f;
    public Transform targetTrackerObject;

    // state control and tracking
    private float altitude = 1f;
    private Quaternion currentRotation = new Quaternion();
	private Quaternion targetRotation = new Quaternion();
	private Vector3 currentLocation = new Vector3();
    public Vector3 targetLocation = new Vector3(0f, 1f, 0f);
    private float[] motorOmega = { 0f, 0f, 0f, 0f };
    private float[] motorReactionTorque = { 0f, 0f, 0f, 0f };

    // --- PID STUFF
    public float[] rollPitchInnerPIDCoeffs = new float[1];
	public float[] rollPitchOuterPIDCoeffs = new float[3];
    public float[] yawPIDCoeffs = new float[3];
    public float[] altPIDCoeffs = new float[4];
    public float[] thrustPIDCoeffs = new float[2];
	public float[] vxzPIDCoeffs = new float[3];
	public float[] xzPIDCoeffs = new float[3];
    private Vector3 errVec = new Vector3();
    PIDFController altitudeController = new PIDFController(20f, 0f, 8f);
	PIDFController thrustController = new PIDFController(kp: 0.3f, kf: 0.4596f);
	PIDFController rollOuterController = new PIDFController(10f, 0f, 2f);
    PIDFController pitchOuterController = new PIDFController(10f, 0f, 2f);
    PIDFController yawController = new PIDFController(1f, 0f, 0.1f);
    PIDFController rollInnerController = new PIDFController(kp: 0.04f);
    PIDFController pitchInnerController = new PIDFController(kp: 0.04f);
	PIDFController vxController = new PIDFController(40f, 0f, 8f);	// input is velocity, output is rotation
	PIDFController vzController = new PIDFController(40f, 0f, 8f);
    public float vxRotationPCoeff = 0.1f;
    PIDFController xController = new PIDFController(4f, 0f, 8f);
	PIDFController zController = new PIDFController(4f, 0f, 8f);
	public float xzLocCoeff = 0.1f;

    // tracking for telemetry
    private float[] motorThrusts = new float[4];
    private float rollPIDCalc = 0f;
	private float pitchPIDCalc = 0f;
    private float yawPIDCalc = 0f;
    private float targetThrustAltitude = 0f;            // the thrust needed to control altitude
    private float targetAngVelRoll = 0f;                // the angular velocity (rad) needed to control roll
    private float targetAngVelPitch = 0f;               // the angular velocity (rad) needed to control pitch
	private float targetAngVelYaw = 0f;
	private float currentTargetVX = 0f;
	private float currentTargetVZ = 0f;
    private Vector3 currentTargetLoc = new Vector3();
	private float targetRotationX = 0f;					// the rotation (roll) required to achieve the desired vx (left/right)
	private float targetRotationZ = 0f;                 // the rotation (pitch) required to achieve the desired vz (forwards/backwards)
    private Vector3 targetVelocity = new(0f, 0f, 0f);
    private float vx = 0f;
    private float vz = 0f;
	private Vector3 targetLocationLocal = new Vector3();

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

	// frequency = frame rate
	void Update() {
        if (targetTrackerObject != null) {
            targetTrackerObject.position = targetLocation;
        }

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

        yawController.Kp = yawPIDCoeffs[0];
		yawController.Ki = yawPIDCoeffs[1];
		yawController.Kd = yawPIDCoeffs[2];

		vxController.Kp = vxzPIDCoeffs[0];
        vxController.Ki = vxzPIDCoeffs[1];
        vxController.Kd = vxzPIDCoeffs[2];
        vzController.Kp = vxzPIDCoeffs[0];
        vzController.Ki = vxzPIDCoeffs[1];
        vzController.Kd = vxzPIDCoeffs[2];

		xController.Kp = xzPIDCoeffs[0];
        xController.Ki = xzPIDCoeffs[1];
        xController.Kd = xzPIDCoeffs[2];
        zController.Kp = xzPIDCoeffs[0];
        zController.Ki = xzPIDCoeffs[1];
        zController.Kd = xzPIDCoeffs[2];

        float yaw = currentRotation.eulerAngles.y;
        if (velocityControlMode) {
            // velocity
            if (Keyboard.current.wKey.isPressed) {
                targetVelocity.z = Mathf.Cos(yaw) * velCtrlModeMaxSpeed;
                targetVelocity.x = Mathf.Sin(yaw) * velCtrlModeMaxSpeed;
            } else if (Keyboard.current.sKey.isPressed) {
                targetVelocity.z = Mathf.Cos(yaw) * -velCtrlModeMaxSpeed;
                targetVelocity.x = Mathf.Sin(yaw) * -velCtrlModeMaxSpeed;
            } else {
				targetVelocity.z = 0f;
                targetVelocity.x = 0f;
			}
            
            if (Keyboard.current.dKey.isPressed) targetVelocity.x = Mathf.Cos(yaw) * velCtrlModeMaxSpeed;
            else if (Keyboard.current.aKey.isPressed) targetVelocity.x = Mathf.Cos(yaw) * -velCtrlModeMaxSpeed;
            else targetVelocity.x = 0f;

			// altitude
            if (Keyboard.current.spaceKey.isPressed) targetLocation.y += ctrlModeAscentSpeed;
            if (Keyboard.current.shiftKey.isPressed) targetLocation.y -= ctrlModeAscentSpeed;

            // yaw
            if (Keyboard.current.eKey.isPressed) targetAngVelYaw = yawSpeed;
            else if (Keyboard.current.qKey.isPressed) targetAngVelYaw = -yawSpeed;
            else targetAngVelYaw = 0f;
		} else if (rotationControlMode) {
            targetRotation = Quaternion.Euler(0f, 0f, 0f);
            if (Keyboard.current.wKey.isPressed) targetRotation = Quaternion.Euler(rotCtrlModeRotAmt, 0f, 0f);
            else if (Keyboard.current.sKey.isPressed) targetRotation = Quaternion.Euler(-rotCtrlModeRotAmt, 0f, 0f);
            if (Keyboard.current.aKey.isPressed) targetRotation = Quaternion.Euler(0f, 0f, rotCtrlModeRotAmt) * targetRotation;
            else if (Keyboard.current.dKey.isPressed) targetRotation = Quaternion.Euler(0f, 0f, -rotCtrlModeRotAmt) * targetRotation;

			// altitude
			if (Keyboard.current.spaceKey.isPressed) targetLocation.y += ctrlModeAscentSpeed;
			if (Keyboard.current.shiftKey.isPressed) targetLocation.y -= ctrlModeAscentSpeed;

			// yaw
			if (Keyboard.current.eKey.isPressed) targetAngVelYaw = yawSpeed;
			else if (Keyboard.current.qKey.isPressed) targetAngVelYaw = -yawSpeed;
			else targetAngVelYaw = 0f;
		} else if (positionControlMode) {
            // home button
            if (Keyboard.current.hKey.isPressed) targetLocation = new Vector3(0f, 1f, 0f);
		}
    }

	// frequency = 50Hz
	void FixedUpdate() {
        float dt = Time.deltaTime;
        // state updates
        altitude = rb.position.y;
        currentRotation = rb.rotation;
		vx = rb.linearVelocity.x;
		vz = rb.linearVelocity.z;
		currentLocation = rb.position;
        currentTargetLoc = new Vector3(
            rampSetpoint(currentTargetLoc.x, targetLocation.x, maxSPIcreaseLoc, dt),
            rampSetpoint(currentTargetLoc.y, targetLocation.y, maxSPIcreaseLoc, dt),
            rampSetpoint(currentTargetLoc.z, targetLocation.z, maxSPIcreaseLoc, dt)
        );
		targetLocationLocal = rb.transform.InverseTransformPoint(currentTargetLoc);

        // calculate equivalent RPMs
        for (int i = 0; i < numMotors; i++) {
            motorOmega[i] = Mathf.Sqrt(motorThrusts[i] / thrustToOmegaCoeff);
        }

		// super super outer loops (target location -> target velocity)
        if (positionControlMode) {
            targetVelocity.z = xzLocCoeff * zController.Update(targetLocationLocal.z, 0, dt);   // location error compared as local
            targetVelocity.x = xzLocCoeff * xController.Update(targetLocationLocal.x, 0, dt);
        }
         
		// super outer loops (target velocity -> target rotation)
        if (positionControlMode || velocityControlMode){
            float cmdX = vxController.Update(currentTargetVX, vx, dt);
            float cmdZ = vzController.Update(currentTargetVZ, vz, dt);

            float yawDeg = target
        }



        // outer loops
        targetThrustAltitude = altitudeController.Update(currentTargetLoc.y, altitude, dt);  // outer controller 
        errVec = makeErrorVec(targetRotation, currentRotation);
        targetAngVelRoll = rollOuterController.Update(errVec.z, dt);
        targetAngVelPitch = pitchOuterController.Update(errVec.x, dt);

        // setpoint ramps
		currentTargetVX = rampSetpoint(currentTargetVX, targetVelocity.x, maxSPIncreaseVel, dt);
        currentTargetVZ = rampSetpoint(currentTargetVZ, targetVelocity.z, maxSPIncreaseVel, dt);

		// inner loops
		float altitudePIDCalc = thrustController.Update(targetThrustAltitude, motorThrusts.Sum() / 4f, dt);
		rollPIDCalc = rollInnerController.Update(targetAngVelRoll, rb.angularVelocity.z, dt);
		pitchPIDCalc = pitchInnerController.Update(targetAngVelPitch, rb.angularVelocity.x, dt);
        yawPIDCalc = yawController.Update(targetAngVelYaw, rb.angularVelocity.y, dt);
		motorThrusts[0] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc + rollPIDCalc - pitchPIDCalc + yawPIDCalc * motorSpinDirection[0]));
        motorThrusts[1] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc - rollPIDCalc - pitchPIDCalc + yawPIDCalc * motorSpinDirection[1]));
        motorThrusts[2] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc + rollPIDCalc + pitchPIDCalc + yawPIDCalc * motorSpinDirection[2]));
        motorThrusts[3] = Mathf.Max(0f, Mathf.Min(1f, altitudePIDCalc - rollPIDCalc + pitchPIDCalc + yawPIDCalc * motorSpinDirection[3]));

		for (int i = 0; i < numMotors; i++) {
			setMotorPower(motorThrusts[i], i);
		}

		// reaction force simulation for yaw
		for (int i = 0; i < numMotors; i++) {
			float reactionForce = omegaToReactionCoeff * Mathf.Pow(motorOmega[i], 2) * motorSpinDirection[i];
			motorReactionTorque[i] = reactionForce;
			rb.AddRelativeTorque(new Vector3(0f, reactionForce, 0f));
		}
	}

	void OnGUI() {
        if (showFullTelemetry) {
            // col 1
            GUI.BeginGroup(new Rect(10f, 10f, 350f, 140f), styleGrayBG);
            GUI.Label(new Rect(10, 10, 300, 25), "TargThrust (Alt): " + targetThrustAltitude.ToString("F2"), styleGreenText);
            for (int i = 0; i < numMotors; i++)
            {
                GUI.Label(new Rect(10, 10 + (i + 1) * 25, 300, 25), "M" + i + " thrust: " + motorThrusts[i].ToString("F2"), styleGreenText);
            }
            GUI.EndGroup();

            GUI.BeginGroup(new Rect(10, 200, 350, 140), styleGrayBG);
            GUI.Label(new Rect(10, 10, 200, 20), "Roll: " + currentRotation.eulerAngles.z.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 35, 200, 20), "Pitch: " + currentRotation.eulerAngles.x.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 60, 200, 20), "Yaw: " + currentRotation.eulerAngles.y.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 85, 200, 20), "Target Pitch: " + targetRotationZ.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 110, 200, 20), "Target Roll: " + targetRotationX.ToString("F2"), styleGreenText);
            GUI.EndGroup();

            GUI.BeginGroup(new Rect(10, 360, 350, 90), styleGrayBG);
            GUI.Label(new Rect(10, 10, 200, 20), "RollErr (rad): " + errVec.z.ToString("F3"), styleGreenText);
            GUI.Label(new Rect(10, 35, 200, 20), "PitchErr (rad): " + errVec.x.ToString("F3"), styleGreenText);
            GUI.Label(new Rect(10, 60, 200, 20), "YawErr (rad): " + errVec.y.ToString("F3"), styleGreenText);
            GUI.EndGroup();

            GUI.BeginGroup(new Rect(10, 460, 350, 70), styleGrayBG);
            GUI.Label(new Rect(10, 10, 200, 20), "Roll Target AngVel: " + targetAngVelRoll.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 35, 200, 20), "Pitch Target AngVel: " + targetAngVelPitch.ToString("F2"), styleGreenText);
            GUI.EndGroup();

            GUI.BeginGroup(new Rect(10, 550, 350, 70), styleGrayBG);
            GUI.Label(new Rect(10, 10, 200, 20), "Roll Thrust: " + rollPIDCalc.ToString("F3"), styleGreenText);
            GUI.Label(new Rect(10, 35, 200, 20), "Pitch Thrust: " + pitchPIDCalc.ToString("F3"), styleGreenText);
            GUI.EndGroup();

			GUI.BeginGroup(new Rect(10, 650, 350, 115), styleGrayBG);
			for (int i = 0; i < numMotors; i++) {
                GUI.Label(new Rect(10, 10 + i * 25, 200, 20), $"M{i} reaction force: {motorReactionTorque[i]:F4}", styleGreenText);
            }
			GUI.EndGroup();

			// col 2
			GUI.BeginGroup(new Rect(400, 10, 350, 90), styleGrayBG); 
            GUI.Label(new Rect(10, 10, 200, 20), "Target Alt: " + targetLocation.y.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 35, 200, 20), "Ramped Target Alt: " + currentTargetLoc.y.ToString("F3"), styleGreenText);
            GUI.Label(new Rect(10, 60, 200, 20), "Current Alt: " + altitude.ToString("F2"), styleGreenText);
            GUI.EndGroup();

            GUI.BeginGroup(new Rect(400, 120, 350, 170), styleGrayBG);
            GUI.Label(new Rect(10, 10, 200, 20), "vx: " + vx.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 35, 200, 20), "vz: " + vz.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 60, 200, 20), "target vx: " + targetVelocity.x.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 85, 200, 20), "target vz: " + targetVelocity.z.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 110, 200, 20), "ramped target vx: " + currentTargetVX.ToString("F3"), styleGreenText);
            GUI.Label(new Rect(10, 135, 200, 20), "ramped target vz: " + currentTargetVZ.ToString("F3"), styleGreenText);
            GUI.EndGroup();

			GUI.BeginGroup(new Rect(400, 320, 350, 170), styleGrayBG);
            GUI.Label(new Rect(10, 10, 200, 20), $"Yaw target angvel: {targetAngVelYaw}", styleGreenText);
			GUI.Label(new Rect(10, 35, 200, 20), $"Yaw vel: {rb.angularVelocity.y}", styleGreenText);
			GUI.Label(new Rect(10, 60, 200, 20), $"Yaw PID output: {yawPIDCalc}", styleGreenText);
			GUI.EndGroup();
		}

        if (showLocationTelemetry) {
            // col 3
            GUI.BeginGroup(new Rect(790, 10, 350, 170), styleGrayBG);
            GUI.Label(new Rect(10, 10, 200, 20), "x: " + currentLocation.x.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 35, 200, 20), "y: " + currentLocation.y.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 60, 200, 20), "z: " + currentLocation.z.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 85, 200, 20), "target x: " + targetLocation.x.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 110, 200, 20), "target y: " + targetLocation.y.ToString("F2"), styleGreenText);
            GUI.Label(new Rect(10, 135, 200, 20), "target z: " + targetLocation.z.ToString("F2"), styleGreenText);
            GUI.EndGroup();
        }
        
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
		Quaternion err = Quaternion.Inverse(current) * target;
		err.ToAngleAxis(out float angle, out Vector3 axis);
		if (angle > 180f) angle -= 360f;
		return axis.normalized * angle * Mathf.Deg2Rad;     // stores 
	}

	private Quaternion Negate(Quaternion q) {
		return new Quaternion(-q.x, -q.y, -q.z, -q.w);
	}
}
