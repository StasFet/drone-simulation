using UnityEngine;

public class SplineVisualiser : MonoBehaviour {

    public Material material;
    public Color colour;
    public float width;
    public float segmentLength = 0.1f;
	public Vector2[] controlPoints;
	public float y;

    void Start() {
        CatmullRomSpline spline = new CatmullRomSpline(controlPoints);
        Debug.Log("spline length is " + spline.SplineLength);

        LineRenderer lineRenderer = gameObject.AddComponent<LineRenderer>();
        lineRenderer.material = new Material(material);
        lineRenderer.startColor = colour;
        lineRenderer.endColor = lineRenderer.startColor;

        lineRenderer.startWidth = width;
        lineRenderer.endWidth = lineRenderer.startWidth;

        lineRenderer.positionCount = Mathf.FloorToInt(spline.SplineLength / segmentLength);

        for (int i = 0; i < lineRenderer.positionCount; i++) {
            Vector2 point = spline.GetLocationByArcLengthNaive(i * segmentLength);
            lineRenderer.SetPosition(i, new Vector3(point.x, y, point.y));
            Debug.Log("Drew new point at: " + point);
        }
    }
}
