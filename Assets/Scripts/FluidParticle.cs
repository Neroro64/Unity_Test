using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FluidParticle : MonoBehaviour {
    const float InteractionRadius = 1f;
    [System.NonSerialized]
    public int defaultParticleDensity = 19;
    public float DENSITY = 997;
    
    public Vector3 tempPos, currentV, tempV, pressure;
    public FluidParticle[] nearbyParticles;
    public double freeSurfaceTerm;

    double lambda;
    public double[] weights;
    double d_ParticleDensity;

    public Vector3 force;

    private void Start()
    {
        freeSurfaceTerm = defaultParticleDensity * 0.97f;
        currentV = new Vector3();
        pressure = new Vector3();
        force = new Vector3(0, -9.8f, 0);
    }

    public Vector3 pos()
    {
        return transform.localPosition;
    }

    public void checkNearbyObjects()
    {
        Collider[] colliders = Physics.OverlapSphere(pos(), InteractionRadius);
        Debug.Log(colliders.Length);
        for (int i = 0; i < colliders.Length; i++)
        {
            Debug.Log(colliders[i].gameObject.name);
            colliders[i].GetComponent<MeshRenderer>().material.color = Color.red;
        }
    }

    public void Initate()
    {
        Collider[] colliders = Physics.OverlapSphere(pos(), InteractionRadius);
        nearbyParticles = new FluidParticle[colliders.Length];
        weights = new double[nearbyParticles.Length];
        nearbyParticles[0] = this;

        double a = 0;
        double b = 0;

        for (int i = 0, j = 1; i < nearbyParticles.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
                
            nearbyParticles[j] = colliders[i].GetComponent<FluidParticle>();
            weights[j] = weightKernel(nearbyParticles[j].pos());
            a += weights[j] * Mathf.Pow((nearbyParticles[j].pos() - this.pos()).sqrMagnitude, 2); // w(r) * r^2
            b += weights[j]; // w(r)
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

    double weightKernel(Vector3 pos)
    {
        return (double)InteractionRadius / (double)(pos - this.pos()).sqrMagnitude;
    }

    public double calcRightSideValue(int i)
    {
        if (i == 0)
            return 0;
        return ((-lambda * DENSITY) / (6f * Time.deltaTime)) * (weights[i] - d_ParticleDensity);
    }

    private void calcLambda()
    {
        double a = 0;
        double b = 0;
        double wr;
        for (int i = 1; i < nearbyParticles.Length; i++)
        {
            wr = weightKernel(nearbyParticles[i].pos());
            a += wr * Mathf.Pow((nearbyParticles[i].pos() - this.pos()).sqrMagnitude, 2); // w(r) * r^2 
            b += wr;
        }

        lambda = a / b;
    }

    void calcWeights()
    {
        for (int i = 1; i < nearbyParticles.Length; i++)
        {
            weights[i] = weightKernel(nearbyParticles[i].pos());
        }
    }

    public void UpdateForce()
    {
        force = currentV / Time.deltaTime + Vector3.Scale(currentV, currentV) + (1 / DENSITY) * pressure;
    }




}
