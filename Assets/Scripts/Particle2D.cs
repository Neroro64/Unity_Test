using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

public abstract class Particle2D : MonoBehaviour {
    public abstract Vector2 Pos();
    public int ID;
    public Vector2 VELOCITY;
    public Vector2 PRESSURE;
    public double PND;
    public double freeSurfaceTerm;
    public Particle2D[] nearbyParticles;

    public abstract void UpdateForce();
    public abstract void Initiate(ref SparseMatrix A, ref Vector<double> B, ref Vector<double> X, int k);
    public abstract void UpdateVelocity();
}
