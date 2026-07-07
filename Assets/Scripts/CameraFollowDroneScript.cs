using UnityEngine;

public class CameraFollowDroneScript : MonoBehaviour {
    public Transform drone;
    public Vector3 offset;

    // Update is called once per frame
    void LateUpdate() {
        if (drone == null) return;
        transform.position = drone.position + offset;
    }
}
