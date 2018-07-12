using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

public class ParticleSpawn : MonoBehaviour {
    public int NumberOfParticles = 800;
    public GameObject ParticlePrefab;
    public string pName = "P0";
    
    private FluidParticle[] particles;
    private SparseMatrix A;
    private Vector<double> B;
    private Vector<double> X;
    private Vector3 resultP = new Vector3();
    private Vector3 temp;
    private Func<int, int, double> calcWeight;
    private Func<int, double> calcResult;

    private bool start = false;

    
    private void Start()
    {
        particles = new FluidParticle[NumberOfParticles];
        
        Vector3 pos = new Vector3();
        int x = 0;
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                for (int k = 0; k < 10; k++)
                {
                    pos.Set(0.8f*j, 0.8f*i, 0.8f*k);
                    particles[x] = Instantiate<GameObject>(ParticlePrefab, transform).GetComponent<FluidParticle>();
                    particles[x].gameObject.transform.localPosition = pos;
                    particles[x].gameObject.name = "P" + ( x++);
                }
            }
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown("space"))
        {
            GameObject.Find(pName).GetComponent<FluidParticle>().checkNearbyObjects();
        }

        else if (Input.GetKeyDown("s")) {
            start = !start;
        }
        if(start)
            calc();
    }
    

    void calc()
    {
        FluidParticle p;
        for (int i = 0; i < particles.Length; i++)
        {
            p = particles[i];
            p.tempV = p.currentV + Time.deltaTime * p.force;
            p.tempPos = p.pos() + p.tempV * Time.deltaTime;

            p.Initate();
            
            if ((p.nearbyParticles.Length - 1) < p.freeSurfaceTerm)
                p.pressure.Set(0, 0, 0);
            else
                p.pressure = calcPressure(p);

            p.currentV = p.tempV + (-Time.deltaTime / p.DENSITY) * resultP;
            p.transform.localPosition += p.currentV * Time.deltaTime;
            p.UpdateForce();
        }
    }

    Vector3 calcPressure(FluidParticle p)
    {
        calcWeight = p.CalcMatrixValue;
        calcResult = p.calcRightSideValue;
        A = SparseMatrix.Create(p.nearbyParticles.Length, p.nearbyParticles.Length, calcWeight);
        B = Vector<double>.Build.Dense(p.nearbyParticles.Length, calcResult);
        X = Vector<double>.Build.Dense(p.nearbyParticles.Length);

        //A.Solve(B, X);
        X = Solve(A, B);
        resultP.Set(0, 0, 0);
        for (int i = 1; i < p.nearbyParticles.Length; i++)
        {
            temp = p.nearbyParticles[i].pos() - p.pos();
            resultP = resultP + ((float)( ( (X.At(i) - X.At(0)) / temp.sqrMagnitude) * p.weights[i]) ) * temp;
        }

        resultP = resultP * (3 / p.defaultParticleDensity);

        return resultP;
    }

    private Vector<double> Solve(SparseMatrix A, Vector<double> B)
    {
        Vector<double> R = Vector<double>.Build.DenseOfVector(B);
        Vector<double> R1;
        Vector<double> P = Vector<double>.Build.DenseOfVector(R);
        int k = 0;

        double a, b;
        Vector<double> X = Vector<double>.Build.Dense(B.ToArray().Length, 0);
        Vector<double> t;
        do
        {
            t = A.Multiply(P);
            a = R.DotProduct(R) / P.DotProduct(t);
            X = X + a * P;
            R1 = R - a * t;
            if (R1.SumMagnitudes() < 0.1f)
                break;
            b = R1.DotProduct(R1) / R.DotProduct(R);
            P = R1 + b * P;
            k++;
        } while (R.SumMagnitudes() - R.At(0)*R.At(0) < 0.1f);

        return X;
    }



}
