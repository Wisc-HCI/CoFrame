using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit;

public class VisualProfilerGetStateScript : MonoBehaviour
{
    [SerializeField]
    private bool startState = false;

    public GameObject background = null;

    private void Start()
    {
        CoreServices.DiagnosticsSystem.ShowProfiler = startState;

        CheckStateInequality();
    }

    private void Update()
    {
        CheckStateInequality();
    }

    private void CheckStateInequality()
    {
        if (CoreServices.DiagnosticsSystem.ShowProfiler != background.activeSelf)
        {
            background.SetActive(CoreServices.DiagnosticsSystem.ShowProfiler);
        }
    }

}
