using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WallParticle : Particle {
    public override Vector3 Pos()
    {
        return transform.localPosition;
    }

}
