using Unity.VectorGraphics;
using UnityEngine;

public class CatmullRomSpline {

	public Vector2[] ControlPoints { get; set; }
	private Vector3[] ctrlPtsDeriv;	// z component stores required derivative

	public CatmullRomSpline() {}
	public CatmullRomSpline (Vector2[] controlPoints) {
		this.ControlPoints = controlPoints;
		Fit();
	}

	public void Fit() {
		if (ControlPoints.Length < 2) return;
		CalculateDerivatives();

		for (int i = 1; i < ctrlPtsDeriv.Length - 1; i++) {
			Vector3 p1 = ctrlPtsDeriv[i - 1];
			Vector3 p2 = ctrlPtsDeriv[i];

			Matrix4x4 gaussianMatrix = new Matrix4x4(
				new Vector4(Mathf.Pow(p1.x, 3), Mathf.Pow(p1.x, 2), p1.x, 1f),
				new Vector4(Mathf.Pow(p2.x, 3), Mathf.Pow(p2.x, 2), p2.x, 1f),
				new Vector4(3f * Mathf.Pow(p1.x, 2), 2f * p1.x, 1f, 0f),
				new Vector4(3f * Mathf.Pow(p2.x, 2), 2f * p2.x, 1f, 0f)
			);
			Vector4 equals = new(p1.y, p2.y, p1.z, p2.z);

			Vector4 coefficients = GaussianElimination4(gaussianMatrix, equals);
		}
	}

	private void CalculateDerivatives() {
		ctrlPtsDeriv = new Vector3[ControlPoints.Length];

		// DERIVATIVE CALCULATIONS
		// first point
		ctrlPtsDeriv[0] = new Vector3(
			ControlPoints[0].x,
			ControlPoints[0].y,
			(ControlPoints[1].y - ControlPoints[0].y) / (ControlPoints[1].x - ControlPoints[0].y)
			);

		// middle points
		for (int i = 1; i < ControlPoints.Length - 1; i++) {
			ctrlPtsDeriv[i] = new Vector3(
				ControlPoints[i].x,
				ControlPoints[i].y,
				(ControlPoints[i + 1].y - ControlPoints[i - 1].y) / (ControlPoints[i + 1].x - ControlPoints[i - 1].x)
			);
		}

		// last point
		int lastIndex = ControlPoints.Length - 1;
		ctrlPtsDeriv[lastIndex] = new Vector3(
			ControlPoints[lastIndex].x,
			ControlPoints[lastIndex].y,
			(ControlPoints[lastIndex].y - ControlPoints[lastIndex - 1].y) / (ControlPoints[lastIndex].x - ControlPoints[lastIndex - 1].x)
		);
	}

	// solving a system of 4 linear equations
	public static Vector4 GaussianElimination4(Matrix4x4 matrix, Vector4 equals) {
		// elimination
		for (int i = 0; i < 3; i++) {           // row at index i will do the elimating (of the elements at index i of each below row)
			SortGaussianMatrix(ref matrix, ref equals, i);
			Vector4 eliminatorRow = matrix.GetRow(i);
			float eliminatorElement = eliminatorRow[i];
			for (int j = i+1; j < 4; j++) {     // row at index j will have an element eliminated
				Vector4 eliminateeRow = matrix.GetRow(j);
				float eliminateeElement = eliminateeRow[i];
				if (eliminateeElement == 0f) continue;  // already eliminated
				float coeff = eliminateeElement / eliminatorElement;
				matrix.SetRow(j, new Vector4(
					eliminateeRow[0] - eliminatorRow[0] * coeff,
					eliminateeRow[1] - eliminatorRow[1] * coeff,
					eliminateeRow[2] - eliminatorRow[2] * coeff,
					eliminateeRow[3] - eliminatorRow[3] * coeff
				));
				equals[j] -= equals[i] * coeff;
			}
		}

		// solve for coefficients
		Vector4 coeffs = new Vector4();
		coeffs[3] = equals[3] / matrix[3, 3];
		coeffs[2] = (equals[2] - matrix[2, 3] * coeffs[3]) / matrix[2, 2];
		coeffs[1] = (equals[1] - matrix[1, 3] * coeffs[3] - matrix[1, 2] * coeffs[2]) / matrix[1, 1];
		coeffs[0] = (equals[0] - matrix[0, 3] * coeffs[3] - matrix[0, 2] * coeffs[2] - matrix[0, 1] * coeffs[1]) / matrix[0, 0];
		return coeffs;
	}

	// sorts a part of the matrix to ensure that rows are in ascending order of leading zero elements
	private static void SortGaussianMatrix(ref Matrix4x4 m, ref Vector4 e, int startingRow) {
		int[] leadingZeros = { 0, 0, 0, 0 };
		for (int i = startingRow; i < 4; i++) {
			Vector4 row = m.GetRow(i);
			for (int j = 0; j < 4; j++) {
				if (row[j] == 0f) {
					leadingZeros[i]++;
					continue;
				}
			}
		}

		int swaps = 1;
		while (swaps != 0) {
			swaps = 0;
			for (int i = startingRow+1; i < 4; i++) {
				if (leadingZeros[i] < leadingZeros[i - 1]) {
					int tempIdx = leadingZeros[i];
					leadingZeros[i] = leadingZeros[i - 1];
					leadingZeros[i - 1] = tempIdx;

					Vector4 tempRow = m.GetRow(i);
					m.SetRow(i, m.GetRow(i - 1));
					m.SetRow(i - 1, tempRow);

					(e[i - 1], e[i]) = (e[i], e[i - 1]);
					swaps++;
				}
			}
		}
	}
}