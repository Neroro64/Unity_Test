using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

public class FluidParticle2D : Particle2D {
    const float Re = 0.4f;
    const float DENSITY = 1000f;//1.161f;
    double defaultPND= 14.418576d;
    double lambda = 1.7d;

    Vector2 GRAVITY = new Vector2(0, 9.8f);
    Vector2 tempV, tempPos, force;

    Particle2D[] nearbyParticles;
    double[] weights;

    //constans for calculations
    double w_t, w_t2;
    private void Start()
    {
        VELOCITY = new Vector2();
        tempV = new Vector2();
        tempPos = new Vector2();
        PRESSURE = new Vector2();
        freeSurfaceTerm = defaultPND * 0.97d;

        w_t = 40d / (7d * Mathf.PI * Re * Re);
        w_t2 = 10d / (7d * Mathf.PI * Re * Re);
    }

    public override Vector2 Pos()
    {
        return transform.localPosition;
    }
    
    public override void UpdateForce()
    {
        force = VELOCITY / Time.fixedDeltaTime + (1f / DENSITY) * PRESSURE - GRAVITY;
        tempV = VELOCITY + force * Time.fixedDeltaTime;
        tempPos = Pos();
        transform.localPosition = tempPos + tempV * Time.fixedDeltaTime;
    }
    public override void Initiate(ref SparseMatrix A, ref Vector<double> B, ref Vector<double> X, int k)
    {
        Collider2D[] colliders = Physics2D.OverlapCircleAll(Pos(), Re);
        nearbyParticles = new Particle2D[colliders.Length];
        weights = new double[colliders.Length];
        nearbyParticles[0] = this;

        PND = 0;
        for (int i = 0, j = 1; j < nearbyParticles.Length; i++)
        {
            if (colliders[i].name.Equals(this.name))
                continue;
            nearbyParticles[j] = colliders[j].GetComponent<Particle2D>();
            weights[j] = weightKernel((nearbyParticles[j].Pos() - this.Pos()).magnitude);
            PND += weights[j];

            A.At(k, nearbyParticles[j].ID, weights[j]);
            j++;
        }
        if (PND < freeSurfaceTerm)
        {
            A.At(k, this.ID, -PND);//A.At(k, this.ID, 0);
            X.At(k, 1d);
            B.At(k, 0); // B.At(k,( (-lambda * DENSITY) / (4d * Time.fixedDeltaTime * Time.fixedDeltaTime) ) * (PND - defaultPND) + PND;
        }
        else
        {
            A.At(k, this.ID, -PND);
            X.At(k, 0);
            B.At(k, ((-lambda * DENSITY) / (4d * Time.fixedDeltaTime * Time.fixedDeltaTime)) * (PND - defaultPND));
        }
    }
    
    public override void UpdateVelocity()
    {
        
    }

    double weightKernel(float r)
    {
        float t = r / Re;
        if (r < 0.5f * Re)
            return w_t * (1d - 6d * Mathf.Pow(t, 2) + 6d * Mathf.Pow(t, 3));
        else if (r < Re)
            return w_t2 * Mathf.Pow(2f - 2 * t, 3);
        else
            return 0;
    }
    
}
