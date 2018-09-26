using System;
using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEngine;

public class WallParticle2D : Particle2D {
    const float Re = 0.4f;
    const float DENSITY = 1000f;//1.161f;
    double lambda = 0.0466947090664703d;//0.051715d;
    double w_t, w_t2;

    public void Start()
    {
        defaultPND = 13.6533306596634d;//5.65685d;
        freeSurfaceTerm = defaultPND * 0.97d;

        w_t = 40d / (7d * Mathf.PI * Re * Re);
        w_t2 = 10d / (7d * Mathf.PI * Re * Re);
    }
    public override Vector2 Pos()
    {
        return transform.localPosition;
    }
    public override void UpdatePND()
    {
        return;
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
            A.At(k, this.ID, -PND);
            X.At(k, 1);
            B.At(k, 0);//B.At(k,( (-lambda * DENSITY) / (4d *  Time.fixedDeltaTime) ) * (PND - defaultPND) + PND);
        }
        else
        {
            A.At(k, this.ID, -PND);
            X.At(k, 0);
            B.At(k, ((-lambda * DENSITY) / (4d * Time.fixedDeltaTime)) * (PND - defaultPND));
        }
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
    public override void InitVelocity() { }
    public override void UpdateForce() { }
    public override void UpdateVelocity() { }



}
