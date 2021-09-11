using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;

public class WalkerVelocity : MonoBehaviour {

	public BezierSpline spline;

	public float duration;

	private float progress;
	private Vector3 velocity;
	private Vector3 prevPos;

	
	public bool lookForward;

	private string filename = "Velocity.txt";

	List<Vector3> listOfVelocity;
	
	void Start(){
		listOfVelocity = new List<Vector3>();
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
		Vector3 velocity_spline = spline.GetDirection(progress);
		
		transform.localPosition = position;
		if (lookForward) {
			transform.LookAt(position + spline.GetDirection(progress));
		}

		listOfVelocity.Add(velocity_spline);
	}
	
	void OnApplicationQuit()
	{
		string textToWriteVelocity = "";
		foreach(Vector3 velocity in listOfVelocity){
			 textToWriteVelocity = textToWriteVelocity + Convert.ToSingle(velocity.x) + " " + Convert.ToSingle(velocity.z) + " " + Convert.ToSingle(velocity.y) + "\r\n";
		}
		File.AppendAllText(filename, textToWriteVelocity);
	}
}