using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FluidParticle : Particle{
    const float InteractionRadius = 3f;
    [System.NonSerialized]
    public double defaultParticleDensity = 170.6f;
    public double particleDensity;
    [System.NonSerialized]
    public float DENSITY = 9.48f;
    public Vector3 Gravity;
    
    public Vector3 tempPos, currentV, tempV, pressure;
    public Particle[] nearbyParticles;
    [System.NonSerialized]
    public double freeSurfaceTerm;

    double lambda;
    public double[] weights;
    double d_ParticleDensity;
    [System.NonSerialized]
    public Vector3 force;

    private void Start()
    {
        freeSurfaceTerm = defaultParticleDensity * 0.97d;
        currentV = new Vector3();
        pressure = new Vector3();
        Gravity = new Vector3(0, 9.8f, 0);
    }
    
    public override Vector3 Pos()
    {
        return transform.localPosition;
    }

    public void checkNearbyObjects()
    {
        Collider[] colliders = Physics.OverlapSphere(Pos(), InteractionRadius);
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
    public double CalcFluidDensity(float radius)
    {
        double n = CalcParticleDensity();
        double density = 0;
        Collider[] colliders = Physics.OverlapSphere(Pos(), radius);
        for (int i = 0; i < colliders.Length; i++)
        {
            if(colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            density += weightKernel(colliders[i].transform.localPosition);
        }
        return n / density;
    }
    public void Initate()
    {
        Collider[] colliders = Physics.OverlapSphere(Pos(), InteractionRadius);
        nearbyParticles = new Particle[colliders.Length];
        weights = new double[nearbyParticles.Length];
        nearbyParticles[0] = this;
        
        particleDensity = 0;
        double a = 0;
        double b = 0;

        for (int i = 0, j = 1; j < nearbyParticles.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            nearbyParticles[j] = colliders[i].GetComponent<Particle>();
            weights[j] = weightKernel(nearbyParticles[j].Pos());
            a += weights[j] * Mathf.Pow((nearbyParticles[j].Pos() - this.Pos()).magnitude, 2); // w(r) * r^2
            b += weights[j]; // w(r)
            particleDensity += weights[j];
            j++;
        }

        lambda = a / b;
        d_ParticleDensity = (double)defaultParticleDensity / (double)nearbyParticles.Length;
    }


    public double CalcMatrixValue(int i, int j)
    {
        if (i == 0)
            return (j == 0) ? 0 : -weights[j];
        if (j == 0)
            return -weights[i];
        else if (j == i)
            return weights[i];
        else
            return 0;
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
        return (double)InteractionRadius / (double)((Pos - this.Pos()).magnitude);
    }

    public double calcRightSideValue(int i)
    {
        if (i == 0)
            return 0;
        return ((-lambda * DENSITY) / (6f * Time.fixedDeltaTime)) * (weights[i] - d_ParticleDensity);
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
        //if (gameObject.name.Equals("P132"))
        //    Debug.DebugBreak();
        force = currentV / Time.fixedDeltaTime + (currentV.x+currentV.y+currentV.z) * currentV + (1f / DENSITY) * pressure - Gravity;
        
    }
}
