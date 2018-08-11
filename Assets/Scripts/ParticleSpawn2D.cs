﻿using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double.Solvers;
using MathNet.Numerics.LinearAlgebra.Solvers;
using System.IO;


public class ParticleSpawn2D : MonoBehaviour {
    [Header("Size")]
    public int NumberOfParticles = 0;
    public int Xsize = 0;
    public int Ysize = 0;
    [Header("Prefab")]
    public GameObject ParticlePrefab;
    public string pName = "P0";
    
    private Particle2D[] particles;
    private SparseMatrix A;
    private Vector<double> B;
    private Vector<double> X;
    private Vector2 temp;
    
    private bool start = false;
    

    private void Start()
    {
        particles = new Particle2D[NumberOfParticles];
        
        Vector2 pos = new Vector2();
        int x = 0;
        for (int i = 0; i < Xsize; i++)
        {
            for (int j = 0; j < Ysize; j++)
            {
                    pos.Set(0.2f*j, 0.2f);
                    particles[x] = Instantiate<GameObject>(ParticlePrefab, transform).GetComponent<Particle2D>();
                    particles[x].gameObject.transform.localPosition = pos;
                    particles[x].gameObject.name = "P" + (x);
                    particles[x].GetComponent<Particle>().ID = x;
                    x++;
            }
        }

        //Setting up matrix A and the vectors for pressure calculation 
        A = SparseMatrix.Create(NumberOfParticles, NumberOfParticles, 0);
        B = Vector.Build.Dense(NumberOfParticles);
        X = Vector.Build.Dense(NumberOfParticles, 0);
    }

    private void Update()
    {
        if (Input.GetKeyDown("space"))
        {
            GameObject.Find(pName).GetComponent<FluidParticle2D>().checkNearbyObjects();
        }
        else if (Input.GetKeyDown("p"))
            Debug.Log(GameObject.Find(pName).GetComponent<FluidParticle2D>().CalcParticleDensity());

        else if (Input.GetKeyDown("d"))
            Debug.Log(GameObject.Find(pName).GetComponent<FluidParticle2D>().CalcDensity());
        else if (Input.GetKeyDown("l"))
            Debug.Log(GameObject.Find(pName).GetComponent<FluidParticle2D>().CalcLambda());
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
        Particle2D p;
        int i;

        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];
            p.UpdateForce();
        }
        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];
            p.Initiate(ref A, ref B, ref X, i);
        }

        Solve(A, B, ref X);
        /*for (i = 0; i < particles.Length; i++)
        {
            if (X.At(i) < 0)
                X.At(i, 0);
        }*/
        
        outPutMatrices();
        Debug.DebugBreak();

        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];
            
            if (p.PND < p.freeSurfaceTerm)
                p.PRESSURE.Set(0, 0);
            else
            {
                p.PRESSURE = calcPressure(p);
            }

            p.currentV = p.tempV + (-Time.fixedDeltaTime / p.DENSITY) * p.pressure;
            p.transform.localPosition = p.tempPos + p.currentV * Time.fixedDeltaTime;
        }

        A.Clear();
        //B.Clear();
        //X.Clear();
        
    }

    Vector2 calcPressure(Particle2D p)
    {
        Vector2 result = new Vector2();
        for (int i = 1; i < p.nearbyParticles.Length; i++)
        {
            temp = p.nearbyParticles[i].Pos() - p.Pos();
            result = resultP + ((float)( ((X.At(p.nearbyParticles[i].ID) - X.At(p.ID)) / Mathf.Pow(temp.magnitude,2) * p.weights[i])) * temp);
        }

        result = result * (float)(3d / p.defaultParticleDensity);
        
        
        return result;
    }

    //Conjugate Gradient Iterative Method
    private void Solve(SparseMatrix A, Vector<double> B, ref Vector<double> X)
    {
        int bLength = B.ToArray().Length;
        Vector<double> R = B - A.Multiply(X);
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
            if (Math.Sqrt(RsNew) < 1E-10d)
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
