using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

public class MatrixTest : MonoBehaviour {
    double count = 0;
    delegate double createValue(int i, int j, out double v);
	// Use this for initialization
	void Start () {
        /*SparseMatrix A = SparseMatrix.OfRowArrays(new double[7][]
        {
            new double[] {0, -1, -1, -1, -1, -1, -1},
            new double[] {-1, 1, 0, 0, 0, 0, 0},
            new double[] {-1, 0, 1, 0, 0, 0, 0},
            new double[] {-1, 0, 0, 1, 0, 0, 0},
            new double[] {-1, 0, 0, 0, 1, 0, 0},
            new double[] {-1, 0, 0, 0, 0, 1, 0},
            new double[] {-1, 0, 1, 0, 0, 0, 1}
        });*/

        Func<int, int, double> init  = cValues;
        SparseMatrix A = SparseMatrix.Create(7, 7, init);

        double lambda = -993f / (6f * 0.01f);
        lambda *= 1f - 5f / 7f;
        double nDefault = 5f;
        VectorBuilder<double> Builder = Vector<double>.Build;
        Vector<double> B = Builder.DenseOfArray(new double[7]
        {
            0,
            lambda,
            lambda,
            lambda,
            lambda,
            lambda,
            lambda,
        });

        VectorBuilder<double> resultB = Vector<double>.Build;
        Vector<double> result = resultB.Dense(7);

        A.Solve(B, result);

        Debug.Log("Matrix A is : " + A.ToString());
        Debug.Log("Matrix B is : " + B.ToString());
        Debug.Log("Lambda is : " + lambda);
        Debug.Log("The result vector3 is : " + result.ToString());

        
	}

    double cValues (int i, int j)
    {
        return count++;
        /*if (i == 0)
            return (j == 0) ? 0 : -1f;
        else if (j == 0)
            return -1f;
        else if (i == j)
            return 1f;
        else
            return 0;
        */

    }

	
}
