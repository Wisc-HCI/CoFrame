using Microsoft.MixedReality.Toolkit;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities.Solvers;

public class HandMenuEnableSolversScript : MonoBehaviour
{
    public bool DISABLED = true;

    public HandBounds handBounds;
    public SolverHandler solverHandler;
    public HandConstraintPalmUp handConstraintPalmUp;

    private void Start()
    {
        handBounds.enabled = !Application.isEditor && !DISABLED;
        solverHandler.enabled = !Application.isEditor && !DISABLED;
        handConstraintPalmUp.enabled = !Application.isEditor && !DISABLED;  
    }

}
