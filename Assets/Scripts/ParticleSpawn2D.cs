﻿using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEditor;
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
    [Header("Wall Size")]
    public int Xsize_wall = 0;
    public int Ysize_wall = 0;
    private int NumOfWallParticles = 0;
    [Header("Prefab")]
    public GameObject ParticlePrefab;
    public GameObject WallPrefab;
    public GameObject WallParticlePrefab;
    public GameObject DummyParticlePrefab;
    public string pName = "P0";
    
    private Particle2D[] particles;
    private SparseMatrix A;
    private Vector<double> B;
    private Vector<double> X;
    private Vector2 temp;

    int t = 0;
    private bool start = false;
    

    private void Start()
    {
        NumOfWallParticles = 2 * Ysize_wall + Xsize_wall - 2;
        particles = new Particle2D[NumberOfParticles + NumOfWallParticles];
        
        Vector2 pos = new Vector2();
        int x = 0;
        for (int i = 0; i < Xsize; i++)
        {
            for (int j = 0; j < Ysize; j++)
            {
                    pos.Set(0.2f*i, 0.2f*j);
                    particles[x] = Instantiate<GameObject>(ParticlePrefab, transform).GetComponent<Particle2D>();
                    particles[x].gameObject.transform.localPosition = pos;
                    particles[x].gameObject.name = "P" + (x);
                    particles[x].ID = x;
                    x++;
            }
        }
        /*
        GameObject dummy;
        for (int i = 0; i < Xsize_wall; i++)
        {
            pos.Set(0.2f * i - 0.2f, -0.2f);
            particles[x] = Instantiate<GameObject>(WallParticlePrefab, WallPrefab.transform).GetComponent<Particle2D>();
            particles[x].gameObject.transform.localPosition = pos;
            particles[x].gameObject.name = "W" + x;
            particles[x].ID = x;
            x++;
            //Dummy
            pos.Set(0.2f * i - 0.2f, -0.4f);
            dummy = Instantiate<GameObject>(DummyParticlePrefab, WallPrefab.transform);
            dummy.transform.localPosition = pos;
            pos.Set(0.2f * i - 0.2f, -0.6f);
            dummy = Instantiate<GameObject>(DummyParticlePrefab, WallPrefab.transform);
            dummy.transform.localPosition = pos;


        }
        for (int i = 1; i < Ysize_wall; i++)
        {
            pos.Set(-0.2f, 0.2f * i - 0.2f);
            particles[x] = Instantiate<GameObject>(WallParticlePrefab, WallPrefab.transform).GetComponent<Particle2D>();
            particles[x].gameObject.transform.localPosition = pos;
            particles[x].gameObject.name = "W" + x;
            particles[x].ID = x;
            x++;
            //Setting up dummy
            pos.Set(-0.4f, 0.2f * i - 0.2f);
            dummy = Instantiate<GameObject>(DummyParticlePrefab, WallPrefab.transform);
            dummy.transform.localPosition = pos;
            pos.Set(-0.6f, 0.2f * i - 0.2f);
            dummy = Instantiate<GameObject>(DummyParticlePrefab, WallPrefab.transform);
            dummy.transform.localPosition = pos;

            pos.Set(Xsize_wall*0.2f - 0.4f, 0.2f * i - 0.2f);
            particles[x] = Instantiate<GameObject>(WallParticlePrefab, WallPrefab.transform).GetComponent<Particle2D>();
            particles[x].gameObject.transform.localPosition = pos;
            particles[x].gameObject.name = "W" + x;
            particles[x].ID = x;
            x++;
            //dummy
            pos.Set(Xsize_wall * 0.2f - 0.2f, 0.2f * i - 0.2f);
            dummy = Instantiate<GameObject>(DummyParticlePrefab, WallPrefab.transform);
            dummy.transform.localPosition = pos;
            pos.Set(Xsize_wall * 0.2f, 0.2f * i - 0.2f);
            dummy = Instantiate<GameObject>(DummyParticlePrefab, WallPrefab.transform);
            dummy.transform.localPosition = pos;

        }*/

        //Setting up matrix A and the vectors for pressure calculation 
        A = SparseMatrix.Create(NumberOfParticles + NumOfWallParticles, NumberOfParticles + NumOfWallParticles, 0);
        B = Vector.Build.Dense(NumberOfParticles+NumOfWallParticles);
        X = Vector.Build.Dense(NumberOfParticles + NumOfWallParticles, 0);
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
        {
            calc();
            if (t++ % 5 == 0)
            {
                EditorApplication.isPaused = true;
                outPutMatrices();
            }
        }
    }


    void calc()
    {
        Particle2D p;
        int i;

        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];
            p.InitVelocity();
        }
        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];
            p.UpdatePND();
            p.Initiate(ref A, ref B, ref X, i);
        }

        Solve(A, B, ref X);
        for (i = 0; i < particles.Length; i++)
        {
            if(X.At(i) < 0)
                X.At(i, 0);
        }


        for (i = 0; i < particles.Length; i++)
        {
            p = particles[i];

            //if (p.PND < p.freeSurfaceTerm)
            //p.PRESSURE.Set(0, 0);
            //else
            //{
            p.PRESSURE = calcPressure(p);
            //}

            p.UpdateForce();
            p.UpdateVelocity();
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
            result = result + ((float)( ((X.At(p.nearbyParticles[i].ID) - X.At(p.ID)) / Mathf.Pow(temp.magnitude,2) * p.WEIGHTS[i])) * temp);
        }

        result = result * (float)(2d / p.defaultPND);
        //if (p.ID == 415)
            //(Debug.DebugBreak();

        return result;
    }

    //Conjugate Gradient Iterative Method
    private void Solve(SparseMatrix A, Vector<double> B, ref Vector<double> X)
    {
        Vector<double> R = B - A.Multiply(X);
        double RsNew;
        double RsOld;
        Vector<double> P = Vector<double>.Build.DenseOfVector(R);
        int k = 0;

        double a, b;
        Vector<double> t;

        RsOld = R.DotProduct(R);
        for (int i = 0; i < NumberOfParticles; i++)
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
