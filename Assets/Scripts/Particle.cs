using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class Particle : MonoBehaviour {
    public abstract Vector3 Pos();
    public int ID;
    public Vector3 tempPos, currentV, tempV, pressure;
}
