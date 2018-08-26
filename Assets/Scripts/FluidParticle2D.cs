using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

public class FluidParticle2D : Particle2D {
    const float Re = 0.4f;
    const float DENSITY = 1000f;//1.161f;
    double lambda = 0.0466947090664703d;//0.051715d;
    Vector2 tempV, tempPos, force;
    
    //constans for calculations
    double w_t, w_t2;
    private void Start()
    {
        VELOCITY = new Vector2();
        tempV = new Vector2();
        tempPos = new Vector2();
        PRESSURE = new Vector2();
        force = -GRAVITY;
        defaultPND = 13.6533306596634d;//5.65685d;
        freeSurfaceTerm = defaultPND * 0.97d;

        w_t = 40d / (7d * Mathf.PI * Re * Re);
        w_t2 = 10d / (7d * Mathf.PI * Re * Re);
    }

    public override Vector2 Pos()
    {
        return transform.localPosition;
    }
    
    public override void InitVelocity()
    {
        tempV = VELOCITY + force * Time.fixedDeltaTime;
        tempPos = Pos();
        transform.localPosition = tempPos + tempV * Time.fixedDeltaTime;
    }
    public override void UpdateForce()
    {
        force = VELOCITY / Time.fixedDeltaTime + (1f / DENSITY) * PRESSURE;
       
    }
    public override void Initiate(ref SparseMatrix A, ref Vector<double> B, ref Vector<double> X, int k)
    {
        Collider2D[] colliders = Physics2D.OverlapCircleAll(Pos(), Re);
        nearbyParticles = new Particle2D[colliders.Length];
        WEIGHTS = new double[colliders.Length];
        nearbyParticles[0] = this;

        PND = 0;
        for (int i = 0, j = 1; j < nearbyParticles.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            nearbyParticles[j] = colliders[i].GetComponent<Particle2D>();
            WEIGHTS[j] = weightKernel((nearbyParticles[j].Pos() - this.Pos()).magnitude);
            PND += WEIGHTS[j];

            A.At(k, nearbyParticles[j].ID, WEIGHTS[j]);
            j++;
        }
        if (PND < freeSurfaceTerm)
        {
            A.At(k, this.ID, 0);//A.At(k, this.ID, -PND);
            X.At(k, 0);
            B.At(k,( (-lambda * DENSITY) / (4d *  Time.fixedDeltaTime) ) * (PND - defaultPND) + PND);
        }
        else
        {
            A.At(k, this.ID, -PND);
            X.At(k, 0);
            B.At(k, ((-lambda * DENSITY) / (4d *  Time.fixedDeltaTime)) * (PND - defaultPND));
        }
    }
    
    public override void UpdateVelocity()
    {
        VELOCITY = tempV + -Time.fixedDeltaTime / DENSITY * PRESSURE;
        transform.localPosition = tempPos + VELOCITY * Time.fixedDeltaTime;
    }

    double weightKernel(float r)
    {
        float t = r / Re;
        //Debug.DebugBreak();
        if (r >= Re)
            return 0;
        if (r < 0.5f * Re && r >= 0)
            return w_t * (1d - 6d * Mathf.Pow(t, 2) + 6d * Mathf.Pow(t, 3));
        else
            return w_t2 * Mathf.Pow(2f - 2 * t, 3);
            
        /*if (r >= Re || r <= 1e-8)
            return 0;
        else
            return Re / r - 1d;
        */
    }

    public void checkNearbyObjects()
    {
        Collider2D[] colliders = Physics2D.OverlapCircleAll(Pos(), Re);
        Debug.Log(colliders.Length);
        for (int i = 0; i < colliders.Length; i++)
        {
            colliders[i].GetComponent<SpriteRenderer>().color = Color.red;
        }
    }
    public double CalcParticleDensity()
    {
        Collider2D[] colliders = Physics2D.OverlapCircleAll(Pos(), Re);
        double particleDensity = 0;
        for (int i = 0; i < colliders.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            particleDensity += weightKernel((colliders[i].GetComponent<Particle2D>().Pos() - Pos()).magnitude);
        }
        return particleDensity;
    }
    public double CalcDensity()
    {
        double n = CalcParticleDensity();

        Collider2D[] colliders = Physics2D.OverlapCircleAll(Pos(), 1f);
        double density = 0;
        for (int i = 0; i < colliders.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            density += weightKernel((colliders[i].GetComponent<Particle2D>().Pos() - Pos()).magnitude);
        }
        return n / density;
    }
    public double CalcLambda()
    {
        Collider2D[] colliders = Physics2D.OverlapCircleAll(Pos(), Re); // maybe unit volume
        double weight, a = 0, b = 0;
        float diff;
        for (int i = 0; i < colliders.Length; i++)
        {
            if (colliders[i].gameObject.name.Equals(this.gameObject.name))
                continue;
            diff = (colliders[i].GetComponent<Particle2D>().Pos() - Pos()).magnitude;
            weight = weightKernel(diff);
            a += Mathf.Pow(diff, 2) * weight;
            b += weight;
        }
        return a / b;

    }

}
