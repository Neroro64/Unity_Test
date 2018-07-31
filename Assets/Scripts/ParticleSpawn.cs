﻿using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Solvers;
using System.IO;


public class ParticleSpawn : MonoBehaviour {
    [System.NonSerialized]
    public int NumberOfParticles = 100;
    public GameObject ParticlePrefab;
    public string pName = "P0";
    
    private FluidParticle[] particles;
    private SparseMatrix A;
    private Vector<double> B;
    private Vector<double> X;
    private Vector<double> values;
    private Vector3 resultP = new Vector3();
    private Vector3 temp;
    private Func<int, double> calcWeight;
    private Func<int, double> calcResult;

    private bool start = false;
    private Vector<double> X1;


    private void Start()
    {
        particles = new FluidParticle[NumberOfParticles];
        
        Vector3 pos = new Vector3();
        int x = 0;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 5; j++)
            {
                for (int k = 0; k < 5; k++)
                {
                    pos.Set(0.1f * j, 0.1f * i, 0.1f * k);
                    particles[x] = Instantiate<GameObject>(ParticlePrefab, transform).GetComponent<FluidParticle>();
                    particles[x].gameObject.transform.localPosition = pos;
                    particles[x].gameObject.name = "P" + ( x);
                    particles[x].GetComponent<Particle>().ID = x;
                    x++;
                }
            }
        }

        A = SparseMatrix.Create(NumberOfParticles, NumberOfParticles, 0);
        B = Vector.Build.Dense(NumberOfParticles);
        X = Vector.Build.Dense(NumberOfParticles, 0);
        //values = Vector.Build.Dense(NumberOfParticles, 0);
    }

    private void Update()
    {
        if (Input.GetKeyDown("space"))
        {
            GameObject.Find(pName).GetComponent<FluidParticle>().checkNearbyObjects();
        }
        else if (Input.GetKeyDown("p"))
            Debug.Log(GameObject.Find(pName).GetComponent<FluidParticle>().CalcParticleDensity());

        else if (Input.GetKeyDown("d"))
            Debug.Log(GameObject.Find(pName).GetComponent<FluidParticle>().CalcDensity());
        else if (Input.GetKeyDown("l"))
            Debug.Log(GameObject.Find(pName).GetComponent<FluidParticle>().CalcLambda());
        else if (Input.GetKeyDown("s"))
        {
            start = !start;
        }
    }

    private void FixedUpdate()
    {
        if (start)
            calc();
    }


    void calc()
    {
        FluidParticle p;
        int i;

        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];

            p.UpdateForce();
            p.tempV = p.currentV + Time.fixedDeltaTime * p.force;
            p.tempPos = p.Pos();

            p.transform.localPosition +=  p.tempV * Time.fixedDeltaTime;
        }
        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];
            p.Initate(ref A, ref B, i);
            //A.SetRow(i, values);
        }
        //X = A.Solve(B);
        B = Vector.Build.Dense(NumberOfParticles, 1d);
        //outPutMatrices();
        //Debug.DebugBreak();
        X = A.Multiply(B);
        Debug.Log(X.DotProduct(X));
        //Solve(A, B, ref X);
        //X = A.SolveIterative(B, new BiCgStab());
        outPutMatrices();
        Debug.DebugBreak();
        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];
            resultP.Set(0, 0, 0);
            if (p.particleDensity < p.freeSurfaceTerm)
                p.pressure = resultP;
            else
            {
                Debug.DebugBreak();
                p.pressure = calcPressure(p);
            }

            p.currentV = p.tempV + (-Time.fixedDeltaTime / p.DENSITY) * p.pressure;
            p.transform.localPosition = p.tempPos + p.currentV * Time.fixedDeltaTime;
        }

        A.Clear();
        //B.Clear();
        X.Clear();
        
    }

    Vector3 calcPressure(FluidParticle p)
    {
        //calcWeight = p.CalcMatrixValue;
        //calcResult = p.calcRightSideValue;
        
        //A = SparseMatrix.Create(p.nearbyParticles.Length-1, p.nearbyParticles.Length-1, calcWeight);
        //B = Vector<double>.Build.Dense(p.nearbyParticles.Length-1, calcResult);
        //X = Vector<double>.Build.Dense(p.nearbyParticles.Length);
        //X = Solve(A, B, X);
        //X = A.Solve(B);

        /* calcWeight = p.CalcMatrixValue2;
        calcResult = p.calcRightSideValue2;
        A = SparseMatrix.Create(p.nearbyParticles.Length-1, p.nearbyParticles.Length, calcWeight);
        B = Vector<double>.Build.Dense(p.nearbyParticles.Length-1, calcResult);
        */ // = Vector<double>.Build.Dense(p.nearbyParticles.Length);
        
        //X = A.SolveIterative(B, new BiCgStab());
        
        for (int i = 1; i < p.nearbyParticles.Length; i++)
        {
            temp = p.nearbyParticles[i].Pos() - p.Pos();
            resultP = resultP + ((float)( ((X.At(p.nearbyParticles[i].ID) - X.At(p.ID)) / Mathf.Pow(temp.magnitude,2) * p.weights[i])) * temp);
        }

        resultP = resultP * (float)(3d / p.defaultParticleDensity);
        
        
        return resultP;
    }

    //Conjugate Gradient Iterative Method
    private void Solve(SparseMatrix A, Vector<double> B, ref Vector<double> X)
    {
        int bLength = B.ToArray().Length;
        Vector<double> R = Vector<double>.Build.DenseOfVector(B);
        double RsNew;
        double RsOld;
        Vector<double> P = Vector<double>.Build.DenseOfVector(R);
        int k = 0;

        double a, b;
        Vector<double> t;

        RsOld = R.DotProduct(R);
        for (int i = 0; i < bLength; i++)
        {
            t = A.Multiply(P);
            a = RsOld / P.DotProduct(t);
            X = X + a * P;
            R = R - a * t;
            RsNew = R.DotProduct(R);
            if (Math.Sqrt(RsNew) < 1E-10f)
                break;
            b = RsNew / RsOld;
            P = R + b * P;
            RsOld = RsNew;
            k++;
        }
    }

    void outPutMatrices()
    {
        using (
            var writer = new StreamWriter(File.Open(Path.Combine(Application.persistentDataPath, "Matrix data.txt"), FileMode.Create))
        )
        {
            int e = 0;
            writer.Write("Matrix A:" + Environment.NewLine);
            for (int o = 0; o < NumberOfParticles; o++)
            {
                for (int w = 0; w < NumberOfParticles; w++)
                {
                    writer.Write(A.At(o, w).ToString("#.###0") + " ");
                }
                writer.Write(Environment.NewLine);
            }

            writer.Write("Vector B is:" + Environment.NewLine);
            for (int o = 0; o < NumberOfParticles; o++)
            {
                writer.Write(B.At(o).ToString("#.###0") + " ");
            }
            writer.Write(Environment.NewLine + "Vector X is:" + Environment.NewLine);
            for (int o = 0; o < NumberOfParticles; o++)
            {
                writer.Write(X.At(o).ToString("#.##0") + " ");
            }
        }
    }



}
