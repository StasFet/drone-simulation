using System.Collections;
using UnityEngine;

public class NewMonoBehaviour : MonoBehaviour {
	public Vector3 pointA;
	public Vector3 pointB;

	// Use this for initialization
	void Start() {
		Vector3[] points = { pointA, pointB };
		CatmullRomSpline spline = new(points);

	}
}