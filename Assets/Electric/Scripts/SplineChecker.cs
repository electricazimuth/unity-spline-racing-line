using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using Azimuth;

#if UNITY_EDITOR
using UnityEditor;
#endif



public class SplineChecker : MonoBehaviour
{
    public bool debugCurvature = true;
    public bool debugBounds = true;
    [SerializeField]
    protected SplineContainer s_Container;
    [SerializeField]
    protected Spline u_Spline; //Unity Spline

    [SerializeField]
    protected Spline offsetSpline;

    public SplineContainer outputSpline;

    public float offset;
    public int offsetResolution;


    public float newTension;

    public float reduceDistance;

    void Awake() {
        InitSpline();
    }

    protected void InitSpline(){
        if (s_Container == null){
            s_Container = GetComponent<SplineContainer>();
        }

        u_Spline = s_Container.Spline;

        
    }

    public void MakeWorkingCopy(){
        InitSpline();
        offsetSpline = u_Spline;
        offsetSpline.Warmup();
    }

    public void OffsetSpline(){
        Debug.Log("CheckSpline");
        InitSpline();
        offsetSpline = SplineHelper.OffsetSpline(offsetSpline, offset, 2);
        
    }

    public void SmoothSpline(){

        InitSpline();
        offsetSpline = SplineHelper.RealSmoothSpline(offsetSpline);

        offsetSpline.Warmup();

    }

    public void Spherise(){

        InitSpline();
        offsetSpline = SplineHelper.Spherise(offsetSpline);

        offsetSpline.Warmup();

    }

    public void Center(){

        InitSpline();
        offsetSpline = SplineHelper.CenterAroundZeroZero(offsetSpline);
        offsetSpline.Warmup();
    }

    

    public void DistanceMerge(){ 
        InitSpline();

        offsetSpline = SplineHelper.ReduceByDistance(offsetSpline, reduceDistance);

        offsetSpline.Warmup();


    }



    public void ModifyTension(){ 
        InitSpline();

        offsetSpline = SplineHelper.ModifyTension(offsetSpline, newTension);
        offsetSpline.Warmup();

    }

    public void CopyToOutSpline(){ 
        InitSpline();
        outputSpline.Spline = offsetSpline;
    }
    




#if UNITY_EDITOR

    protected void OnDrawGizmosSelected(){

        Vector3 textOffset = Vector3.one * 0.1f;
        Vector3 knotPos;
        
        if(debugCurvature && offsetSpline.Count > 0){

            GUIStyle debug_style = new GUIStyle(); 
            debug_style.normal.textColor = Color.cyan;

            float numDivisions = 400f;
            float increment = 1f / numDivisions;

            Gizmos.color = Color.blue;
            float lineSize = 1f;

            for(float i = 0f; i <= 1f; i += increment){

                offsetSpline.Evaluate(i, out float3 p_position, out float3 p_tangent, out float3 p_upVector);
                float curvature = offsetSpline.EvaluateCurvature(i);
                Gizmos.DrawLine(p_position, p_position + (SplineHelper.NormalFromTangent(p_tangent,p_upVector) * curvature * lineSize));

            }

            debug_style.normal.textColor = Color.red;

            for(int i = 0; i < offsetSpline.Count; i++){
                knotPos = (Vector3)offsetSpline[i].Position;
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere( transform.position + knotPos, 1f);
                Handles.Label(transform.position + knotPos + textOffset + (textOffset * 0.1f * i), i.ToString(), debug_style );
            }
            debug_style.normal.textColor = Color.cyan;
            for(int i = 0; i < u_Spline.Count; i++){
/*
                float lerpAmount = (float)i / (float)u_Spline.Count;
                Color lerpedColor = Color.Lerp(Color.green, Color.yellow, lerpAmount );
                lerpedColor.a = 0.66f;
                Gizmos.color = lerpedColor;
*/
                knotPos = (Vector3)u_Spline[i].Position;
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere( transform.position + knotPos, 1f);
                Handles.Label(transform.position + knotPos + textOffset + (textOffset * 0.1f * i), i.ToString(), debug_style );

            }
        }

        if( debugBounds ){

            Bounds outerBounds = SplineHelper.GetSplineOuterBounds( offsetSpline, 25 );

            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(outerBounds.center,  outerBounds.size );


            Bounds innerBounds = SplineHelper.GetSplineInnerBounds(offsetSpline, 25);
            Gizmos.color = Color.white;
            Gizmos.DrawWireCube(innerBounds.center,  innerBounds.size );


            Bounds knotBounds = SplineHelper.GetKnotBounds(offsetSpline);
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireCube(knotBounds.center,  knotBounds.size );





        }
    }

#endif
}
