using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(FPSCounter))]
public class FPSDisplay : MonoBehaviour {
	public Text fpsLabel;

	FPSCounter fpsCounter;
	void Awake(){
		fpsCounter = GetComponent<FPSCounter>();
	}	
	void Update(){
		// fpsLabel.text = Mathf.Clamp(fpsCounter.AverageFPS, 0, 999).ToString();
		fpsLabel.text = fpsCounter.AverageFPS.ToString();
	}
}
