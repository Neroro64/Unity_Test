using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

public class FluidParticle : Particle{
    const float InteractionRadius = 2.1f;
    [System.NonSerialized]
    public double defaultParticleDensity = 14.418576f;
    public double particleDensity;
    [System.NonSerialized]
    public float DENSITY = 1.161f;
    public Vector3 Gravity;
    
    public Particle[] nearbyParticles;
    [System.NonSerialized]
    public double freeSurfaceTerm;

    double lambda =1.7d;
    public double[] weights;
    double d_ParticleDensity;
    [System.NonSerialized]
    public Vector3 force;

    private double term1 , term2;

    private void Start()
    {
        freeSurfaceTerm = defaultParticleDensity * 0.97d;
        currentV = new Vector3();
        pressure = new Vector3();
        Gravity = new Vector3(0, 9.8f, 0);
        term1 = 6d / (lambda * defaultParticleDensity);
        term2 = -DENSITY / (Time.fixedDeltaTime*Time.fixedDeltaTime);
    }
    
    public override Vector3 Pos()
    {
        return transform.localPosition;
    }

    public void checkNearbyObjects()
    {
        Collider[] colliders = Physics.OverlapCircleAll(Pos(), InteractionRadius);
        Debug.Log(colliders.Length);
        for (int i = 0; i < colliders.Length; i++)
        {
            Debug.Log(colliders[i].gameObject.name);
            colliders[i].GetComponent<MeshRenderer>().material.color = Color.red;
        }
    }
    public double CalcParticleDensity()
    {
        Collider[] colliders = Physics.OverlapSphere(Pos(), InteractionRadius);
        double particleDensity = 0;
        for (int i = 0; i < colliders.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            particleDensity += weightKernel(colliders[i].transform.localPosition);
        }
        return particleDensity;
    }
    public double CalcDensity()
    {
        double n = CalcParticleDensity();

        Collider[] colliders = Physics.OverlapSphere(Pos(), 1f);
        double density = 0;
        for (int i = 0; i < colliders.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            density += weightKernel(colliders[i].transform.localPosition);
        }
        return n / density;
    }

    public double CalcLambda()
    {
        Collider[] colliders = Physics.OverlapSphere(Pos(), InteractionRadius); // maybe unit volume
        double weight, a = 0, b = 0;
        for (int i = 0; i < colliders.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            weight = weightKernel(colliders[i].transform.localPosition);
            a += Mathf.Pow((colliders[i].transform.localPosition - this.Pos()).magnitude, 2) * weight;
            b += weight;
        }
        return a / b;

    }
    
    public void Initate(ref SparseMatrix A, ref Vector<double> B, ref Vector<double> X,  int k) 
    {
        Collider[] colliders = Physics.OverlapSphere(Pos(), InteractionRadius);
        nearbyParticles = new Particle[colliders.Length];
        weights = new double[nearbyParticles.Length];
        nearbyParticles[0] = this;
        
        particleDensity = 0;
        
        for (int i = 0, j = 1; j < nearbyParticles.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            nearbyParticles[j] = colliders[i].GetComponent<Particle>();
            weights[j] = weightKernel(nearbyParticles[j].Pos());
            particleDensity += weights[j];

            A.At(k, nearbyParticles[j].ID, weights[j]);
            j++;
        }
        if (particleDensity < freeSurfaceTerm)
        {
            A.At(k, this.ID, 0);
            X.At(k, 1d);
            //B.At(k, term2 * ((particleDensity - defaultParticleDensity) / defaultParticleDensity) +
            //    ((6d*particleDensity) / (lambda*defaultParticleDensity)));
            B.At(k, (-lambda * DENSITY * (particleDensity - defaultParticleDensity)) / (6d  * Time.fixedDeltaTime*Time.fixedDeltaTime) + particleDensity);
            //B.At(k, 0);
        }
        else
        {
            A.At(k, this.ID, -particleDensity);
            X.At(k, 0);
            //B.At(k, term2 * ((particleDensity - defaultParticleDensity) / defaultParticleDensity));

            B.At(k, (-lambda * DENSITY * (particleDensity - defaultParticleDensity)) / (6d * Time.fixedDeltaTime * Time.fixedDeltaTime));
        }


    }


    public double CalcMatrixValue(int i)
    {
        return weights[i + 1];
    }
    public double CalcMatrixValue2(int i, int j)
    {
        if (j == 0)
            return -weights[i+1];
        else if (j-1 == i)
            return weights[i+1];
        else
            return 0;
    }

    double weightKernel(Vector3 Pos)
    {
        float r = (Pos - this.Pos()).magnitude;
        if (r >= InteractionRadius)
            return 0;
        else
            return (double)InteractionRadius / (double)r - 1d;
    }

    public double calcRightSideValue(int i)
    {
        return ((-lambda * DENSITY) / (6f * Time.fixedDeltaTime*Time.fixedDeltaTime)) * (weights[i+1] - d_ParticleDensity);
    }
    public double calcRightSideValue2(int i)
    {
        return ((-lambda * DENSITY) / (6f * Time.fixedDeltaTime)) * (weights[i+1] - d_ParticleDensity);
    }


    private void calcLambda()
    {
        double a = 0;
        double b = 0;
        double wr;
        for (int i = 1; i < nearbyParticles.Length; i++)
        {
            wr = weightKernel(nearbyParticles[i].Pos());
            a += wr * Mathf.Pow((nearbyParticles[i].Pos() - this.Pos()).magnitude, 2); // w(r) * r^2 
            b += wr;
        }

        lambda = a / b;
    }

    void calcWeights()
    {
        for (int i = 1; i < nearbyParticles.Length; i++)
        {
            weights[i] = weightKernel(nearbyParticles[i].Pos());
        }
    }

    public void UpdateForce()
    {

        //if (gameObject.name.Equals("P31"))
          //  Debug.DebugBreak();
        force = currentV / Time.fixedDeltaTime  + (1f / DENSITY) * pressure - Gravity;

        
    }

    private float calcVelocityGradient()
    {
        double result = 0;
        Particle p;
        for (int i = 1; i < nearbyParticles.Length; i++)
        {
            p = nearbyParticles[i];
            result += Vector3.Dot((p.currentV - this.currentV), (p.Pos() - this.Pos())) /
                (p.Pos() - this.Pos()).magnitude;
            result *= weights[i];
        }
        return (float) (result * (6d / defaultParticleDensity));
    }
}
