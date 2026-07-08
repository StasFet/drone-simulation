using Unity.VectorGraphics;
using UnityEngine;

public class CatmullRomSpline {

	public Vector2[] ControlPoints { get; set; }
	private Vector3[] ctrlPtsDeriv;				// z component stores required derivative
	public Vector4[] cubicCoeffs;               // cubicCoeffs[i] holds the coefficients for the cubic spline from ControlPoints[i] to ControlPoints[i+1]
	private static float IntegralStepSize = 0.001f;
	public float SplineLength;
	private float[] cumSegmentArcLengths;	// cumSegmentArcLengths[i] holds the arc length of all segments until ControlPoints[i]
	private float[] segmentArcLengths;		// segmentsArcLengths[i] is the arc length of the segment between ControlPoints[i] and ControlPoints[i+1]

	public CatmullRomSpline() {}
	public CatmullRomSpline (Vector2[] controlPoints) {
		ControlPoints = controlPoints;
		Fit();
	}

	public void Fit() {
		if (ControlPoints.Length < 2) return;
		CalculateDerivatives();
		Debug.Log($"Calculated Derivatives: \n{ctrlPtsDeriv[0]}\n{ctrlPtsDeriv[1]}");

		cubicCoeffs = new Vector4[ControlPoints.Length - 1];
		for (int i = 0; i < ctrlPtsDeriv.Length - 1; i++) {
			Vector4 p1 = ctrlPtsDeriv[i];
			Vector4 p2 = ctrlPtsDeriv[i + 1];

			Matrix4x4 gaussianMatrix = new Matrix4x4(
				new Vector4(Mathf.Pow(p1.x, 3), Mathf.Pow(p1.x, 2), p1.x, 1f),
				new Vector4(Mathf.Pow(p2.x, 3), Mathf.Pow(p2.x, 2), p2.x, 1f),
				new Vector4(3f * Mathf.Pow(p1.x, 2), 2f * p1.x, 1f, 0f),
				new Vector4(3f * Mathf.Pow(p2.x, 2), 2f * p2.x, 1f, 0f)
			).transpose;
			Vector4 equals = new(p1.y, p2.y, p1.z, p2.z);

			Vector4 coefficients = GaussianElimination4(ref gaussianMatrix, ref equals);
			cubicCoeffs[i] = coefficients;
			//Debug.Log($"Coefficients between p{i} and p{i + 1}: {coefficients}");
		}

		GenerateLengthArrays();
	}

	private void CalculateDerivatives() {
		ctrlPtsDeriv = new Vector3[ControlPoints.Length];

		// DERIVATIVE CALCULATIONS
		ctrlPtsDeriv[0] = new Vector3(
			ControlPoints[0].x,
			ControlPoints[0].y,
			(ControlPoints[1].y - ControlPoints[0].y) / (ControlPoints[1].x - ControlPoints[0].x)
		);

		for (int i = 1; i < ControlPoints.Length - 1; i++) {
			ctrlPtsDeriv[i] = new Vector3(
				ControlPoints[i].x,
				ControlPoints[i].y,
				(ControlPoints[i + 1].y - ControlPoints[i - 1].y) / (ControlPoints[i + 1].x - ControlPoints[i - 1].x)
			);
		}

		int lastIndex = ControlPoints.Length - 1;
		ctrlPtsDeriv[lastIndex] = new Vector3(
			ControlPoints[lastIndex].x,
			ControlPoints[lastIndex].y,
			(ControlPoints[lastIndex].y - ControlPoints[lastIndex - 1].y) / (ControlPoints[lastIndex].x - ControlPoints[lastIndex - 1].x)
		);
	}

	private void GenerateLengthArrays() {
		float[] cumLengths = new float[ControlPoints.Length];
		float[] lengths = new float[ControlPoints.Length - 1];
		SplineLength = 0f;
		cumLengths[0] = 0;
		for (int i = 0; i < ctrlPtsDeriv.Length - 1; i++) {
			float arcLength = ArcLength(cubicCoeffs[i], ControlPoints[i].x, ControlPoints[i + 1].x);
			//Debug.Log($"arc length between p{i} and p{i + 1}: {arcLength}");
			SplineLength += arcLength;
			lengths[i] = SplineLength;
			cumLengths[i + 1] = SplineLength;
		}
		cumSegmentArcLengths = cumLengths;
		segmentArcLengths = lengths;
	}

	public Vector2 GetLocationByArcLengthNaive(float arcLength) {
		// binary search to find the segment within which this arcLength lies
		int left = 0;
		int right = segmentArcLengths.Length;
		int mid;

		while (left < right) {
			mid = left + Mathf.FloorToInt((right - left) / 2f);
			if (segmentArcLengths[mid] < arcLength) {
				left = mid + 1;
			} else {
				right = mid;
			}
		}

		// stupid bit here....... it assumes that the arc length is evenly distributed across the domain of the function
		int pointIndex = left;
		Vector2 p1 = ControlPoints[left];
		Vector2 p2 = ControlPoints[left + 1];
		float arcLengthPerX = (segmentArcLengths[Mathf.Min(pointIndex + 1, segmentArcLengths.Length - 1)] - segmentArcLengths[Mathf.Min(pointIndex, segmentArcLengths.Length - 1)]) / Mathf.Abs(p1.x - p2.x);
		float arcLengthRemainder = arcLength - segmentArcLengths[pointIndex];
		float xDistance = arcLengthRemainder / arcLengthPerX;   // the distance in the x direction from p1 to the desired point
		float xValue = p1.x + (p1.x < p2.x ? xDistance : -xDistance);
		Vector4 splineCoeffs = cubicCoeffs[pointIndex];
		float yValue = splineCoeffs[0] * Mathf.Pow(xValue, 3) + splineCoeffs[1] * Mathf.Pow(xValue, 2) + splineCoeffs[2] * xValue + splineCoeffs[3];

		return new Vector2(xValue, yValue);
	}

	// solving a system of 4 linear equations
	public static Vector4 GaussianElimination4(ref Matrix4x4 matrix, ref Vector4 equals) {
		// elimination
		for (int i = 0; i < 3; i++) {           // row at index i will do the elimating (of the elements at index i of each below row)
			SortGaussianMatrix(ref matrix, ref equals, i);
			// LogGaussianMatrix(matrix, equals);
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
			// LogGaussianMatrix(matrix, equals);
		}

		// solve for coefficients
		Vector4 coeffs = new(1f, 1f, 1f, 1f);

		for (int i = 3; i >= 0; i--) {  // row iteration
			float curr = equals[i];
			for (int j = 3; j > i; j--) {	// element iteration
				curr -= matrix[i, j] * coeffs[j];
			}
			coeffs[i] = curr / matrix[i, i];
		}
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
				} else {
					break;
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

	public static void LogGaussianMatrix(Matrix4x4 m, Vector4 v) {
		string total = "";
		for (int i = 0; i < 4; i++) {
			string row = "| ";
			for (int j = 0; j < 4; j++) {
				row += m.GetRow(j)[i].ToString("F2") + ", ";
			}
			row += "| " + v[i].ToString("F2") + " |\n";
			total += row;
		}
		Debug.Log(total);
	}

	public static float ArcLength(Vector4 cubicCoefficients, float x1, float x2) {
		Vector3 derivCoeffs = new(3 * cubicCoefficients[0], 2 * cubicCoefficients[1], cubicCoefficients[2]);

		float lower = Mathf.Min(x1, x2);
		float upper = Mathf.Max(x1, x2);

		// no integration, just a middle riemann sum ;D
		bool invertAtEnd = x1 > x2; // if the lower bound is larger than the upper bound, flip the sign at the end
		float integral = 0;
		for (float i = lower; i < upper; i += IntegralStepSize) {
			float dydx = 0.5f * (
				derivCoeffs[0] * Mathf.Pow(i, 2) +
				derivCoeffs[1] * i +
				derivCoeffs[2] +
				derivCoeffs[0] * Mathf.Pow(i + IntegralStepSize, 2) +
				derivCoeffs[1] * (i + IntegralStepSize) +
				derivCoeffs[2]
			);
			float funcToIntegrate = Mathf.Sqrt(1 + Mathf.Pow(dydx, 2));
			integral += funcToIntegrate * IntegralStepSize;
		}
		return invertAtEnd ? -integral : integral;
	}
}