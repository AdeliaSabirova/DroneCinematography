using UnityEngine;
using System;
using System.IO;
using System.Collections.Generic;

	[ExecuteInEditMode]
	public class testSpline : MonoBehaviour
	{
		public CatmullRom spline;
        private CatmullRom.CatmullRomPoint[] splinepoints;

		public Transform[] controlPoints;

		[Range(2, 30)]
		public int resolution;
		public bool closedLoop;

		[Range(0, 20)]
		public float normalExtrusion;

		[Range(0, 20)]
		public float tangentExtrusion;

		public bool drawNormal, drawTangent;

        private string filename = "DroneCoordinates.txt";

		void Start()
		{
			if(spline == null)
			{
				spline = new CatmullRom(controlPoints, resolution, closedLoop);
			}
		}

		void Update()
		{
			if(spline != null)
			{
				spline.Update(controlPoints);
				spline.Update(resolution, closedLoop);
				spline.DrawSpline(Color.white);

				if(drawNormal)
					spline.DrawNormals(normalExtrusion, Color.red);

				if(drawTangent)
					spline.DrawTangents(tangentExtrusion, Color.cyan);
			}
			else
			{
				spline = new CatmullRom(controlPoints, resolution, closedLoop);
			}
		}

        void OnApplicationQuit()
	    {
		    string textToWrite = "";
            splinepoints = spline.GetPoints();
		
		    foreach(CatmullRom.CatmullRomPoint point in splinepoints){
			     textToWrite = textToWrite + Convert.ToSingle(point.position.x) + " " + Convert.ToSingle(point.position.z) + " " + Convert.ToSingle(point.position.y) + "\r\n";
		    }
		    File.AppendAllText(filename, textToWrite);

	    }
	}

