using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;

public abstract class Particle2D : MonoBehaviour {
    public int ID;
    public Vector2 VELOCITY;
    public Vector2 PRESSURE;
    public Vector2 GRAVITY = new Vector2(0, 9.8f);
    public double PND;
    public double defaultPND; 
    public double freeSurfaceTerm;
    public Particle2D[] nearbyParticles;
    public double[] WEIGHTS;

    public abstract Vector2 Pos();
    public abstract void InitVelocity();
    public abstract void UpdateForce();
    public abstract void Initiate(ref SparseMatrix A, ref Vector<double> B, ref Vector<double> X, int k);
    public abstract void UpdateVelocity();
}
