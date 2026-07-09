using UnityEngine;

public class SplineVisualiser : MonoBehaviour {

    public Material material;
    public Color colour;
    public float width;
    public float segmentLength = 0.1f;
    public float refreshRate = 5f;
    private float dt = 0f;
	public Vector3[] controlPoints;
    private LineRenderer lr;

    void Start() {   
        lr = gameObject.AddComponent<LineRenderer>();
        lr.material = new Material(material);
		lr.startColor = colour;
		lr.endColor = lr.startColor;

		lr.startWidth = width;
		lr.endWidth = lr.startWidth;
    }

	private void Update () {
        dt += Time.deltaTime;
        if (dt > 1f / refreshRate) {
            dt = 0;
			CatmullRomSpline spline = new CatmullRomSpline(controlPoints);
			//Debug.Log("spline length is " + spline.SplineLength);
			lr.positionCount = 0;
			lr.positionCount = Mathf.FloorToInt(spline.SplineLength / segmentLength);

			for (int i = 0; i < lr.positionCount; i++) {
				lr.SetPosition(i, spline.GetLocationByArcLengthNewton(i * segmentLength));
				// Debug.Log($"Point at AL = {i * segmentLength}: {point}");
			}
		}
	}
}
