using UnityEditor;
using UnityEngine;
using UnityEngine.Splines;

[CustomEditor(typeof(SplineChecker))]
public class SplineCheckerEditor : Editor {

    private SplineChecker checker;
	public SplineContainer m_Spline;


    public override void OnInspectorGUI () {

		EditorGUILayout.LabelField("Check and modify a spline", EditorStyles.boldLabel);
		
		EditorGUILayout.HelpBox("Check Spline",
		MessageType.Info, true
		);

		EditorGUILayout.Space();

		if(GUILayout.Button("Make Copy")){

			checker = (SplineChecker)target;
            checker.MakeWorkingCopy();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Offset")){

			checker = (SplineChecker)target;
            checker.OffsetSpline();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("RealSmooth")){

			checker = (SplineChecker)target;
            checker.SmoothSpline();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Distance Merge")){

			checker = (SplineChecker)target;
            checker.DistanceMerge();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Spherise")){

			checker = (SplineChecker)target;
            checker.Spherise();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Center zero, zero")){

			checker = (SplineChecker)target;
            checker.Center();
			
        }


		EditorGUILayout.Space();

		if(GUILayout.Button("Modify Tension")){

			checker = (SplineChecker)target;
            checker.ModifyTension();
			
        }


		EditorGUILayout.Space();

		if(GUILayout.Button("Copy to Out Spline")){

			checker = (SplineChecker)target;
            checker.CopyToOutSpline();
			
        }


		
/*
        EditorGUILayout.Space();

		if(GUILayout.Button("Generate Authored Race Track")){

			checker = (SplineChecker)target;
			SomeEditorOnlyFunction( checker );
            //generator.ExtraDebug();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Regenerate")){

			checker = (SplineChecker)target;
            checker.Regenerate();
			
        }

		EditorGUILayout.Space();
*/
        DrawDefaultInspector();

	}


	public void SomeEditorOnlyFunction(SplineChecker checker){
/*
			RaceTrackAuthoredSpline splineHolder = generator.gameObject.GetComponent<RaceTrackAuthoredSpline>();
			if( splineHolder != null){
				SplineContainer m_Spline = splineHolder.m_Spline;

				if( m_Spline != null){
					generator.SetUSpline ( m_Spline.Spline );
					generator.CleanPrevious();
					generator.GenerateColliders();
					generator.GenerateWaypoints();
					generator.GenerateVisuals();
				}

			}else{
				Debug.Log("RaceTrackGeneratorBaseEditor > GenerateFromAuthoredSpline: splineHolder is NULL");
			}
*/
	}

}

