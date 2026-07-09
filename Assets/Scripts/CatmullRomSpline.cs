using System;
using System.Runtime.CompilerServices;
using Unity.VectorGraphics;
using UnityEngine;

public class CatmullRomSpline {

	public Vector2[] ControlPoints { get; set; }
	private Vector2[] ctrlPtsDeriv;				// z component stores required derivative
	private Func<float, float>[] segmentFunctionsX;
	private Func<float, float>[] segmentFunctionsY;
	private Func<float, float>[] segmentFunctionsXPrime;
	private Func<float, float>[] segmentFunctionsYPrime;
	private static float IntegralSamples = 1000f;
	public float SplineLength;
	private float[] cumSegmentArcLengths;	// cumSegmentArcLengths[i] holds the arc length of all segments until ControlPoints[i]
	private float[] segmentArcLengths;      // segmentsArcLengths[i] is the arc length of the segment between ControlPoints[i] and ControlPoints[i+1]
	private int nPoints;
	private int nSegments;

	public CatmullRomSpline() {}
	public CatmullRomSpline (Vector2[] controlPoints) {
		ControlPoints = controlPoints;
		nPoints = ControlPoints.Length;
		nSegments = nPoints - 1;
		Fit();
	}

	public void Fit() {
		if (ControlPoints.Length < 2) return;
		nPoints = ControlPoints.Length;
		nSegments = nPoints - 1;

		CalculateDerivatives();
		// Debug.Log($"Calculated Derivatives: \n{ctrlPtsDeriv[0]}\n{ctrlPtsDeriv[1]}");
		segmentFunctionsX = new Func<float, float>[nSegments];
		segmentFunctionsY = new Func<float, float>[nSegments];
		segmentFunctionsXPrime = new Func<float, float>[nSegments];
		segmentFunctionsYPrime = new Func<float, float>[nSegments];

		for (int i = 0; i < nSegments; i++) {
			Vector4 p1 = ControlPoints[i];
			Vector4 p2 = ControlPoints[i + 1];
			Vector4 m1 = ctrlPtsDeriv[i];
			Vector4 m2 = ctrlPtsDeriv[i + 1];

			Vector4 equalsX = new(p1.x, p2.x, m1.x, m2.x);
			Vector4 equalsY = new(p1.y, p2.y, m1.y, m2.y);

			Vector4 coefficientsX = CalculateCoefficients(equalsX);
			Vector4 coefficientsY = CalculateCoefficients(equalsY);
			//Debug.Log($"x(t) = {coefficientsX[0]}x^3 + {coefficientsX[1]}x^2 + {coefficientsX[2]}x + {coefficientsX[3]}\n" +
			//	$"y(t) = {coefficientsY[0]}y^3 + {coefficientsY[1]}y^2 + {coefficientsY[2]}y + {coefficientsY[3]}");
			segmentFunctionsX[i] = x => coefficientsX[0] * Mathf.Pow(x, 3) + coefficientsX[1] * Mathf.Pow(x, 2) + coefficientsX[2] * x + coefficientsX[3];
			segmentFunctionsY[i] = y => coefficientsY[0] * Mathf.Pow(y, 3) + coefficientsY[1] * Mathf.Pow(y, 2) + coefficientsY[2] * y + coefficientsY[3];
			segmentFunctionsXPrime[i] = x => 3f * coefficientsX[0] * Mathf.Pow(x, 2) + 2f * coefficientsX[1] * x + coefficientsX[2];
			segmentFunctionsYPrime[i] = y => 3f * coefficientsY[0] * Mathf.Pow(y, 2) + 2f * coefficientsY[1] * y + coefficientsY[2];
			//Debug.Log($"Coefficients between p{i} and p{i + 1}: {coefficients.ToString("F4")}");
		}

		GenerateLengthArrays();
	}

	private void CalculateDerivatives() {
		ctrlPtsDeriv = new Vector2[ControlPoints.Length];

		// DERIVATIVE CALCULATIONS
		ctrlPtsDeriv[0] = (ControlPoints[1] - ControlPoints[0]) / 2f;

		for (int i = 1; i < nPoints - 1; i++) {
			ctrlPtsDeriv[i] = (ControlPoints[i + 1] - ControlPoints[i - 1]) / 2f;
		}

		int lastIndex = ControlPoints.Length - 1;
		ctrlPtsDeriv[lastIndex] = (ControlPoints[lastIndex] - ControlPoints[lastIndex - 1]) / 2f;
	}

	private void GenerateLengthArrays() {
		float[] cumLengths = new float[nPoints];
		float[] lengths = new float[nSegments];
		SplineLength = 0f;
		cumLengths[0] = 0;
		for (int i = 0; i < ctrlPtsDeriv.Length - 1; i++) {
			float arcLength = ArcLength(segmentFunctionsXPrime[i], segmentFunctionsYPrime[i]);
			SplineLength += arcLength;
			lengths[i] = arcLength;
			cumLengths[i + 1] = SplineLength;
		}
		cumSegmentArcLengths = cumLengths;
		segmentArcLengths = lengths;
	}

	public Vector2 GetLocationByArcLengthNewton(float arcLength) {
		// binary search to find the segment within which this arcLength lies
		int left = 0;
		int right = segmentArcLengths.Length;
		int mid;

		while (left < right) {
			mid = left + Mathf.FloorToInt((right - left) / 2f);
			if (cumSegmentArcLengths[mid] < arcLength) {
				left = mid + 1;
			} else {
				right = mid;
			}
		}

		// solve for x value within segment
		// Debug.Log($"The arc length {arcLength} is between p{left - 1} and p{left}");
		int pointIndex = left;
		if (cumSegmentArcLengths[pointIndex] == arcLength) return ControlPoints[pointIndex];   // if the requested arc length is a full number of bluds
		float lengthRemainder = arcLength - cumSegmentArcLengths[pointIndex - 1];
		float guess = lengthRemainder / segmentArcLengths[pointIndex - 1];
		Func<float, float> speed = t => Mathf.Sqrt(Mathf.Pow(segmentFunctionsXPrime[pointIndex - 1](t), 2) + Mathf.Pow(segmentFunctionsYPrime[pointIndex - 1](t), 2));
		float tVal = SolveUpperBound(speed, lengthRemainder, guess) * (lengthRemainder / segmentArcLengths[pointIndex - 1]);
		return new Vector2(segmentFunctionsX[pointIndex - 1](tVal), segmentFunctionsY[pointIndex - 1](tVal));
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

	// solve the specific system of linear equation for the spline parametrised from 0 to 1
	public static Vector4 CalculateCoefficients(Vector4 equals) {
		return new Vector4(
			2f * equals[0] - 2f * equals[1] + equals[2] + equals[3],
			-3f * equals[0] + 3f * equals[1] - 2f * equals[2] - equals[3],
			equals[2],
			equals[0]
		);
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

	public static float ArcLength(Func<float, float> dxdt, Func<float, float> dydt) {
		float stepSize = 1f / IntegralSamples;

		float integral = 0;
		for (float i = 0; i < 1f; i += stepSize) {
			integral += stepSize * Mathf.Sqrt(Mathf.Pow(dxdt(i), 2) + Mathf.Pow(dydt(i), 2));
		}

		return Mathf.Abs(integral);
	}

	float SolveUpperBound(Func<float, float> f, float target, float guess, int numSteps = 20) {
		float T = guess;
		for (int i = 0; i < numSteps; i++) {
			float g = Integral(f, 0f, T) - target;
			float gPrime = f(T);
			float step = g / gPrime;
			T -= step;
			if (Mathf.Abs(step) < 1e-6f) break;
		}
		return T;
	}

	private static float Integral(Func<float, float> f, float a, float b, int n = 100) {
		float h = (b - a) / n;
		float sum = f(a) + f(b);
		for (int i = 1; i < n; i++) {
			sum += f(a + i * h) * (i % 2 == 1 ? 4f : 2f);
		}
		return sum * h / 3f;
	}
}