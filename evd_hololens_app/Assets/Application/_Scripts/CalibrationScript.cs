using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Microsoft.MixedReality.Toolkit;

public class CalibrationScript : MonoBehaviour
{
    public Microsoft.MixedReality.Toolkit.UI.ProgressIndicatorOrbsRotator progress = null;

    public System.Action<bool> callback = null;

    public AudioClip onCalibrateClip = null;

    private float time = 0;

    [SerializeField]
    private new AudioSource audio = null;

    public GameObject marker = null;

    public EnvironmentScript environment = null;

    public void Calibrate(System.Action<bool> callback)
    {
        this.callback = callback;
        time = Time.time;

        progress.OpenAsync();
        progress.Message = "Starting calibration : Please find Marker";
        progress.Progress = 0;
    }

    public void Stop()
    {
        callback = null;
        time = 0;

        progress.Message = "Calibration Cancelled";

        progress.CloseAsync();
    }

    private void Update()
    {
        if (callback != null && Time.time - time > 5)
        {
            progress.Message = "Calibration Successful!";
            progress.Progress = 1;

            audio.PlayOneShot(onCalibrateClip);

            environment.gameObject.SetActive(true);
            environment.CalibratePositionAt(marker.transform);

            if (Time.time - time > 10)
            {
                progress.CloseAsync();

                callback(true);
                callback = null;
                time = 0;
            }        
        }
    }
}
