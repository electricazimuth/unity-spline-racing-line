using UnityEditor;
using UnityEngine;
using UnityEngine.Splines;

[CustomEditor(typeof(RacingLine))]
public class RacingLineEditor : Editor {

    private RacingLine checker;

    public override void OnInspectorGUI () {

		EditorGUILayout.LabelField("Check and modify a spline", EditorStyles.boldLabel);
		
		EditorGUILayout.HelpBox("Check Spline",
		MessageType.Info, true
		);


		EditorGUILayout.Space();

		if(GUILayout.Button("Initialise data")){

			checker = (RacingLine)target;
            checker.Regen();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Calculate Racing Line")){

			checker = (RacingLine)target;
            checker.CalcRacingLine();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Info Print")){

			checker = (RacingLine)target;
            checker.EchoInfo();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Add Waypoint Gameobjects")){

			checker = (RacingLine)target;
            checker.AddWaypointGameObjects();
			
        }

		EditorGUILayout.Space();

		if(GUILayout.Button("Clear Data")){

			checker = (RacingLine)target;
            checker.EmptyData();
			
        }


        DrawDefaultInspector();

	}


	public void SomeEditorOnlyFunction(RacingLine checker){
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

