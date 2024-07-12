using System.Collections;
using System.Collections.Generic;
using System.Text;
using System;
using UnityEngine;
using UnityEditor;
using UnityEngine.Splines;
using Unity.Mathematics;

using UnityRemiCoulom;


public class RacingLine : MonoBehaviour
{
    
    public SplineContainer m_Spline;
    [SerializeField]

    public Transform waypointHolder;

    private Spline u_Spline;
    public float trackWidth = 10f;

    [SerializeField]
    private Spline innerSpline;
    [SerializeField]
    private Spline outerSpline;

    [SerializeField]
    private Spline racingLineSpline;

    [SerializeField]
    private Vector3[] waypoints;

    //[SerializeField]
    //private Vector3[] racingLineWaypoints;

    private float[] midPointOffsets;

    public bool debugLines = true;

    public int knotResolution = 3;

    public int iterations = 3; //smoothing iterations 3 to 4 seems enough

    public float vehicleWidth = 3f;

    public float invertMulti = 1f;

    // Creates waypoints and spline data
    public void Regen()
    {
        u_Spline = m_Spline.Spline;

        int knotsSize = u_Spline.Count;

        innerSpline = new Spline();
        innerSpline.Closed = true;
        outerSpline = new Spline();
        outerSpline.Closed = true;

        racingLineSpline = new Spline();
        racingLineSpline.Closed = true;

        float splinetime = 0;
        int running = 0;

        List<Vector3> waypointsList = new List<Vector3>();

        for(int i = 0; i < knotsSize; i++){
            for(int m = 0; m < knotResolution; m++){ 
                float mid = (float)i + ((float)m / (float)knotResolution);


                float splineTime = u_Spline.CurveToSplineT(mid);

                //Debug.Log(i + " " + m + " = " + mid + " t: " + splineTime);

                u_Spline.Evaluate(splineTime, out var p_position, out var p_tangent, out var p_upVector);

                float3 normalDirection = math.normalize(math.cross(p_tangent, p_upVector));
                float3 offset = trackWidth * 0.5f * normalDirection;//math.normalizesafe(p_tangent);
                
                innerSpline.Insert(running, new BezierKnot( p_position - offset ), TangentMode.AutoSmooth);
                outerSpline.Insert(running, new BezierKnot( p_position + offset ), TangentMode.AutoSmooth);

                racingLineSpline.Insert(running, new BezierKnot(p_position), TangentMode.AutoSmooth);

                waypointsList.Add( p_position );
                //mid_Spline.Insert(running, new BezierKnot( p_position), TangentMode.AutoSmooth);
                running++;
            }
        }

        innerSpline.Warmup() ;
        outerSpline.Warmup() ;

        waypoints = waypointsList.ToArray();

        midPointOffsets = new float[ running ];


    }
/*
    // Update is called once per frame
    public void REMICalcRacingLine()
    {
        Debug.Log("CalcRacingLine");
        int knotsSize = u_Spline.Count;
        // needs to be greater than 128..
        int rlKnotResolution = Mathf.CeilToInt( 128f / (float)knotsSize );
        int size = knotsSize * rlKnotResolution;

        



        //generate the inner / outer points;
        float[] m_txLeft = new float[size];
        float[] m_tyLeft = new float[size];
        float[] m_txRight = new float[size]; 
        float[] m_tyRight = new float[size];

        int k = 0;

        for(int i = 0; i < knotsSize; i++){
            for(int m = 0; m < rlKnotResolution; m++){ 
                
                float mid = (float)i + ((float)m / (float)rlKnotResolution);
                float innerSplineTime = innerSpline.CurveToSplineT(mid);
                float outerSplineTime = outerSpline.CurveToSplineT(mid);

                float3 innerPos = innerSpline.EvaluatePosition(innerSplineTime);
                float3 outerPos = outerSpline.EvaluatePosition(outerSplineTime);

                m_txLeft[k] = outerPos.x;
                m_tyLeft[k] = outerPos.z;
                m_txRight[k] = innerPos.x;
                m_tyRight[k] = innerPos.z;
                k++;
            }
        }

        if( k != size){
            Debug.Log("indexes not matching finsla resolution size! k:" + k + " size: "+size);
            return;
        }

        
        SmoothRacingLine racingLine = new SmoothRacingLine();
        racingLine.SetPerimetersXYData( m_txLeft, m_tyLeft, m_txRight, m_tyRight );
        racingLine.CalcRaceLine();
        racingLineWaypoints = racingLine.GetRacingLineVector3();

        Debug.Log( "racingLineWaypoints size: " + racingLineWaypoints.Length );



        
    }
*/


    public void CalcRacingLine() {

        //int resolution = 4;
        //Debug.Log("Curvature Check");

        float widthLimit = 1f - (vehicleWidth / trackWidth);
        for( int l = 0; l <= iterations; l++){
            for( int i = 0 ; i < racingLineSpline.Count ; i++){

                //get curvature
                int next = i + 1;
                if(next >= racingLineSpline.Count){
                    next = 0;
                }
                int prev = i - 1;
                if(prev < 0){
                    prev = racingLineSpline.Count - 1;
                }

                float prevSplineTime = racingLineSpline.CurveToSplineT(prev);
                float thisSplineTime = racingLineSpline.CurveToSplineT(i);
                float nextSplineTime = racingLineSpline.CurveToSplineT(next);

                //racingLineSpline.Evaluate(splineTime, out var p_position, out var p_tangent, out var p_upVector);
                //racingLineSpline.Evaluate(nextSplineTime, out var n_position, out var n_tangent, out var n_upVector);

                //racingLineSpline.Evaluate(prevSplineTime, out var p_position, out var p_tangent, out var p_upVector);

                float3 p_position = racingLineSpline.EvaluatePosition(prevSplineTime);
                racingLineSpline.Evaluate(thisSplineTime, out var t_position, out var t_tangent, out var t_upVector);
                float3 n_position = racingLineSpline.EvaluatePosition(nextSplineTime);
                //racingLineSpline.Evaluate(nextSplineTime, out var n_position, out var n_tangent, out var n_upVector);

                float rInv = GetRInverse(p_position, t_position, n_position) * invertMulti;

                if( Mathf.Abs(rInv) > 0.00001f ) {

                    
                    float multiplier = rInv;

                    //float multiplier = Mathf.Clamp( rInv * 0.025f , 0f, 1f ) ;
                    if( Mathf.Abs( midPointOffsets[i] + multiplier ) < widthLimit){ //the widthLimit so we don't get too close to the outer points, calculated form the vehicle width
                        //these are -1f to 1f to be multiplied by trackWidth and normal direction and added to the mid point
                        midPointOffsets[i] += multiplier;

                        BezierKnot knot = racingLineSpline[i];

                        float3 normalDirection = math.normalize(math.cross(t_tangent, t_upVector));
                        knot.Position = (float3)waypoints[i] + (midPointOffsets[i] * trackWidth * 0.5f * normalDirection); //offset;
                        racingLineSpline[i] = knot;

                        racingLineSpline.Warmup();

                    }
                
                }
               
            }
        }

        //get lengths of standard waypoint vs racing line
        Debug.Log("Racingline length:" + racingLineSpline.GetLength() + " Waypoint path length:" + u_Spline.GetLength() );

        

    }


    //this one is computationaly overly complex, eg. slower, and not as good (the one above generates smoother lines at lower iterations)
    public void Alt_CalcRacingLine() {

        //int resolution = 4;
        //Debug.Log("Curvature Check");

        float widthLimit = 1f - (vehicleWidth / trackWidth);
        for( int l = 0; l <= iterations; l++){
            for( int i = 0 ; i < racingLineSpline.Count ; i++){

                //get curvature
                int next = i + 1;
                if(next >= racingLineSpline.Count){
                    next = 0;
                }
                int prev = i - 1;
                if(prev < 0){
                    prev = racingLineSpline.Count - 1;
                }

                float nextSplineTime = racingLineSpline.CurveToSplineT(next);
                float prevSplineTime = racingLineSpline.CurveToSplineT(prev);
                float splineTime = racingLineSpline.CurveToSplineT(i);

                racingLineSpline.Evaluate(splineTime, out var p_position, out var p_tangent, out var p_upVector);

                racingLineSpline.Evaluate(nextSplineTime, out var n_position, out var n_tangent, out var n_upVector);

                //get curvatures at each point
                float nextCurvature = racingLineSpline.EvaluateCurvature(nextSplineTime);
                float prevCurvature = racingLineSpline.EvaluateCurvature(prevSplineTime);
                float curvature = racingLineSpline.EvaluateCurvature(splineTime);

                //Vector3 acceleration = (Vector3)racingLineSpline.EvaluateAcceleration(splineTime);

                //float3 tangentDiffs = p_tangent - n_tangent;

                float angle = Vector3.SignedAngle(p_tangent, n_tangent, Vector3.down);

                

                //Get the differnce between current and the interpolated prev+next curvature
                float averaged = (prevCurvature + nextCurvature) * 0.5f;
                float diff = Mathf.Abs(curvature - averaged);

                if( diff > 0.01f ) {

                    float3 normalDirection = math.normalize(math.cross(p_tangent, p_upVector));

                    float turningMultiplier = Mathf.Sign( angle ); //get the angle sign so we know the turn direction 

                    float multiplier = Mathf.Clamp( diff * 0.025f , 0f, 1f ) * turningMultiplier;

                    if( Mathf.Abs( midPointOffsets[i] + multiplier ) < widthLimit){ //the widthLimit so we don't get too close to the outer points, calculated form the vehicle width
                        //these are -1f to 1f to be multiplied by trackWidth and normal direction and added to the mid point
                        midPointOffsets[i] += multiplier;

                        BezierKnot knot = racingLineSpline[i];

                        knot.Position = (float3)waypoints[i] + (midPointOffsets[i] * trackWidth * 0.5f * normalDirection); //offset;
                        racingLineSpline[i] = knot;

                        racingLineSpline.Warmup();

                    }

                }


                
            }
        }

        //get lengths of standard waypoint vs racing line
        Debug.Log("Racing Line Length:" + racingLineSpline.GetLength() + " Waypoint Line Length:" + u_Spline.GetLength() );

        

    }

    public void EchoInfo(){

        for( int i = 0 ; i < racingLineSpline.Count ; i++){
            int next = i + 1;
            if(next >= racingLineSpline.Count){
                next = 0;
            }
            int prev = i - 1;
            if(prev < 0){
                prev = racingLineSpline.Count - 1;
            }

            float nextSplineTime = racingLineSpline.CurveToSplineT(next);
            float prevSplineTime = racingLineSpline.CurveToSplineT(prev);
            float splineTime = racingLineSpline.CurveToSplineT(i);

            racingLineSpline.Evaluate(prevSplineTime, out var p_position, out var p_tangent, out var p_upVector);
            racingLineSpline.Evaluate(splineTime, out var t_position, out var t_tangent, out var t_upVector);
            racingLineSpline.Evaluate(nextSplineTime, out var n_position, out var n_tangent, out var n_upVector);

            float rInv = GetRInverse(p_position, t_position, n_position);

            

            //get curvatures at each point
            float nextCurvature = racingLineSpline.EvaluateCurvature(nextSplineTime);
            float prevCurvature = racingLineSpline.EvaluateCurvature(prevSplineTime);
            float curvature = racingLineSpline.EvaluateCurvature(splineTime);

            //Vector3 acceleration = (Vector3)racingLineSpline.EvaluateAcceleration(splineTime);

            //float3 tangentDiffs = p_tangent - n_tangent;

            float angle = Vector3.SignedAngle(p_tangent, n_tangent, Vector3.down);

            

            //Get the differnce between current and the interpolated prev+next curvature
            float averaged = (prevCurvature + nextCurvature) * 0.5f;
            float diff = Mathf.Abs(curvature - averaged);


        
            //Debug.Log(i + " prev:" + prevCurvature + " this:" + curvature + " next:" + nextCurvature + " diff:" + diff );   
            //Debug.Log(i + " prev:" + prevCurvature + " this:" + curvature + " next:" + nextCurvature + " diff:" + diff );
            //Debug.Log(i +"  tangent:" + ((Vector3)p_tangent).ToString("F1") + " tangent angles:" + angle );
            Debug.Log(i +"===  RInverse :" + rInv + " curvature:" + curvature + " tangent angles:" + angle );

                    
        }

    }

    public void AddWaypointGameObjects(){
        if( waypointHolder != null && waypoints !=null && waypoints.Length > 0){
            for(int i=0; i < waypoints.Length; i++){
                GameObject _waypoint = new GameObject();
                _waypoint.transform.position = waypoints[i];
                _waypoint.name = "waypoint_" + i;
                _waypoint.transform.parent = waypointHolder;

            }
        }else{
            Debug.LogError("Error: no waypoint holder or waypoints generated");
        }
    }

    public void EmptyData(){

        u_Spline = m_Spline.Spline;

        int knotsSize = u_Spline.Count;

        innerSpline = new Spline();
        innerSpline.Closed = true;
        outerSpline = new Spline();
        outerSpline.Closed = true;

        racingLineSpline = new Spline();
        racingLineSpline.Closed = true;

        float splinetime = 0;
        int running = 0;

        List<Vector3> waypointsList = new List<Vector3>();

    }



    /////////////////////////////////////////////////////////////////////////////
	// Compute the inverse of the radius from 3 points - assuming top down x & z co-ordinates
	/////////////////////////////////////////////////////////////////////////////
    private float GetRInverse(float3 p1, float3 p2, float3 p3)
    {
        float x1 = p3.x - p2.x;
        float y1 = p3.z - p2.z;
        float x2 = p1.x - p2.x;
        float y2 = p1.z - p2.z;
        float x3 = p3.x - p1.x;
        float y3 = p3.z - p1.z;

        float det = x1 * y2 - x2 * y1;
        float n1 = x1 * x1 + y1 * y1;
        float n2 = x2 * x2 + y2 * y2;
        float n3 = x3 * x3 + y3 * y3;
        float nnn = Mathf.Sqrt(n1 * n2 * n3);

        float c = 2 * det / nnn;
        return c;
    }

/*
	private float GetRInverse(int prev, float x, float y, int next)
    {
        float x1 = tx[next] - x;
        float y1 = ty[next] - y;
        float x2 = tx[prev] - x;
        float y2 = ty[prev] - y;
        float x3 = tx[next] - tx[prev];
        float y3 = ty[next] - ty[prev];

        float det = x1 * y2 - x2 * y1;
        float n1 = x1 * x1 + y1 * y1;
        float n2 = x2 * x2 + y2 * y2;
        float n3 = x3 * x3 + y3 * y3;
        float nnn = Mathf.Sqrt(n1 * n2 * n3);

        float c = 2 * det / nnn;
        return c;
    }
*/











#if UNITY_EDITOR

    protected void OnDrawGizmosSelected(){

        Vector3 textOffset = Vector3.one * 0.1f;


        if(debugLines && innerSpline != null && outerSpline != null){

            

            for(int i = 0; i < innerSpline.Count; i++){
                   

                    Gizmos.color = Color.red;
                    //Gizmos.DrawLine( innerLastPosition, innerCurrentPosition);
                    int next = (i + 1) % innerSpline.Count;

                    //Gizmos.DrawSphere( transform.position + (Vector3)innerSpline[i].Position, 0.8f);
                    Gizmos.DrawLine( transform.position + (Vector3)innerSpline[i].Position, transform.position + (Vector3)innerSpline[next].Position);

                    //innerLastPosition = innerCurrentPosition;
                    
                

                    //float outerSplineTime = outerSpline.CurveToSplineT(mid);
                //Debug.Log(i + " " + m + " = " + mid + " t: " + splineTime);
                    //u_Spline.Evaluate(splineTime, out var p_position, out var p_tangent, out var p_upVector);
                    //float3 outerCurrentPosition = outerSpline.EvaluatePosition(outerSplineTime);
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine( transform.position + (Vector3)outerSpline[i].Position, transform.position + (Vector3)outerSpline[next].Position);

                    //Gizmos.DrawSphere( transform.position + (Vector3)outerSpline[i].Position, 0.8f);
                    //outerLastPosition = outerCurrentPosition;
          
            }
    

            Gizmos.color = Color.yellow;
            //for(int i = 0; i < u_Spline.Count; i++){
            for(int i = 0; i < waypoints.Length; i++){
                
                //Vector3 knotPos = (Vector3)u_Spline[i].Position;
                Gizmos.DrawSphere( waypoints[i], 0.2f);
                Handles.Label(waypoints[i] + textOffset + (textOffset * 0.1f * i), i.ToString());
            }

/*
            if( racingLineWaypoints != null ){

                Gizmos.color = Color.white;

                for(int i = 0; i < racingLineWaypoints.Length; i++){

                    Gizmos.DrawSphere( transform.position + Vector3.up + (Vector3)racingLineWaypoints[i], 0.4f);

                }

            }
*/
            if( racingLineSpline != null ){

                Gizmos.color = Color.green;
                int rlres = 4;

                
                float t = (float)racingLineSpline.Count - (1f/(float)rlres);
                float3 prevPos = racingLineSpline.EvaluatePosition(t); //float prevt = racingLineSpline.CurveToSplineT(t);


                for(int i = 0; i < racingLineSpline.Count; i++){

                    Gizmos.DrawSphere( transform.position + (Vector3)racingLineSpline[i].Position + Vector3.up , 0.4f);

                    for(int r = 0; r < 4; r++){

                        
                        //int rnext = r + 1;
                        //if(rnext >= racingLineSpline.Count){
                        //    rnext = 0;
                        //}

                        t = (float)i + ((float)r / (float)rlres);
                        float rlSplineTime = racingLineSpline.CurveToSplineT(t);

                        float3 rlPos = racingLineSpline.EvaluatePosition(rlSplineTime);



                        Gizmos.DrawLine( transform.position + (Vector3)prevPos , transform.position + (Vector3)rlPos);

                        prevPos = rlPos;
                    }

                }
            }


    

        }


    }



#endif 


}
