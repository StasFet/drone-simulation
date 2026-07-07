using UnityEngine;

public class CameraFollowDroneScript : MonoBehaviour {
    public Transform drone;
    public Vector3 offset = new Vector3(0.2f, 0.2f, -0.5f);
    public float yawOffset = -20f;
    public float pitch = 15f;

    void LateUpdate() {
        if (drone == null) return;

        float targetYaw = drone.eulerAngles.y;
        float currentYaw = targetYaw + yawOffset;

        Quaternion yawOnlyRotation = Quaternion.Euler(0f, targetYaw, 0f);
        transform.position = drone.position + yawOnlyRotation * offset;
        transform.rotation = Quaternion.Euler(pitch, currentYaw, 0f);
    }
}
