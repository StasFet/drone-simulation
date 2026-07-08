using System.Collections;
using UnityEngine;

public class NewMonoBehaviour : MonoBehaviour {

	public Matrix4x4 matrix;
	public Vector4 equals;
	public Vector4 solutions;

	public Vector2 pointA;
	public Vector2 pointB;

	// Use this for initialization
	void Start() {
		CatmullRomSpline.LogGaussianMatrix(matrix, equals);
		solutions = CatmullRomSpline.GaussianElimination4(ref matrix, ref equals);
		Vector4 p1 = new Vector3(
			pointA.x,
			pointA.y,
			(pointB.y - pointA.y) / (pointB.x - pointA.x)
			//0f
		);
		Vector3 p2 = new Vector3(
			pointB.x,
			pointB.y,
			(pointB.y - pointA.y) / (pointB.x - pointA.x)
			//-2f
		);
		Debug.Log($"p1: {p1}\np2: {p2}");

		Matrix4x4 gaussianMatrix = new Matrix4x4(
				new Vector4(Mathf.Pow(p1.x, 3), Mathf.Pow(p1.x, 2), p1.x, 1f),
				new Vector4(Mathf.Pow(p2.x, 3), Mathf.Pow(p2.x, 2), p2.x, 1f),
				new Vector4(3f * Mathf.Pow(p1.x, 2), 2f * p1.x, 1f, 0f),
				new Vector4(3f * Mathf.Pow(p2.x, 2), 2f * p2.x, 1f, 0f)
			).transpose;
		//gaussianMatrix.
		Vector4 equalz = new(p1.y, p2.y, p1.z, p2.z);
		CatmullRomSpline.LogGaussianMatrix(gaussianMatrix, equalz);
		Vector4 coefficients = CatmullRomSpline.GaussianElimination4(ref gaussianMatrix, ref equalz);
		Debug.Log($"Line between pA and pB: {coefficients[0]}x^3 + {coefficients[1]}x^2 + {coefficients[2]}x + {coefficients[3]}");
		float arcLength = CatmullRomSpline.ArcLength(coefficients, p1.x, p2.x);
		Debug.Log($"Arc length between pA and pB: {arcLength}");

	}
}