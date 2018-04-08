using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FPSCounter : MonoBehaviour {
	public int FPS {get; private set;}
	public int AverageFPS{get; private set;}
	
	public int frameRange = 60;

	int[] fpsBuffer;
	int fpsBufferIndex;

	void initializeBuffer() {
		if (frameRange <= 0)
			frameRange = 1;
		fpsBuffer = new int[frameRange];
		fpsBufferIndex = 0;
	}
	void Update(){
		if (fpsBuffer == null || fpsBuffer.Length != frameRange) {
			initializeBuffer();
		}
		updateBuffer();
		calculateFPS();

		// FPS = (int) (1f / Time.deltaTime);
	}
	void updateBuffer() {
		fpsBuffer[fpsBufferIndex++] = (int) (1f / Time.deltaTime);
		if (fpsBufferIndex >= frameRange) {
			fpsBufferIndex = 0;
		}
	}

	void calculateFPS() {
		int sum = 0;
		for (int i = 0; i<frameRange; i++){
			sum += fpsBuffer[i];
		}
		AverageFPS = sum/frameRange;
	}


}
