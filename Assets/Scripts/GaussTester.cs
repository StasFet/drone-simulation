using System.Collections;
using UnityEngine;

public class NewMonoBehaviour : MonoBehaviour {
	public Vector4 solutions;

	public Vector2 pointA;
	public Vector2 pointB;

	// Use this for initialization
	void Start() {
		Vector2[] points = { pointA, pointB };
		CatmullRomSpline spline = new(points);

	}
}