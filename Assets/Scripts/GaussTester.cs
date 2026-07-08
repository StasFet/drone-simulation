using System.Collections;
using UnityEngine;

public class NewMonoBehaviour : MonoBehaviour {

	public Matrix4x4 matrix;
	public Vector4 equals;
	public Vector4 solutions;

	// Use this for initialization
	void Start() {
		solutions = CatmullRomSpline.GaussianElimination4(ref matrix, ref equals);
	}
}