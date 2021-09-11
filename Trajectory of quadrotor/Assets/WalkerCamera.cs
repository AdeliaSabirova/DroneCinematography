using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;

public class WalkerCamera : MonoBehaviour {

	public BezierSpline spline;

	public float duration;

	private float progress;
	private Vector3 velocity;
	private Vector3 prevPos;

	
	public bool lookForward;

	private string filename = "CameraCoordinates.txt";
	
	List<Vector3> listOfPosition;
	List<Vector3> listOfVelocity;
	
	void Start(){
		listOfPosition = new List<Vector3>();
	}

	void FixedUpdate (){
    		velocity = (transform.position - prevPos)/Time.deltaTime;
    		prevPos = transform.position;
	}

	private void Update () {
		progress += Time.deltaTime / duration;
		if (progress > 1f) {
			progress = 1f;
		}
		Vector3 position = spline.GetPoint(progress);

		transform.localPosition = position;
		if (lookForward) {
			transform.LookAt(position + spline.GetDirection(progress));
		}

		listOfPosition.Add(position);
	}
	
	void OnApplicationQuit()
	{
		string textToWrite = "";
		
		foreach(Vector3 position in listOfPosition){
			 textToWrite = textToWrite + Convert.ToSingle(position.x) + " " + Convert.ToSingle(position.z) + " " + Convert.ToSingle(position.y) + "\r\n";
		}
		File.AppendAllText(filename, textToWrite);
    }
}