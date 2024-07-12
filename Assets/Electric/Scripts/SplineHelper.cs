using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using UnityEngine.Splines;
using Evryway;
using LIR = Evryway.LargestInteriorRectangle;


namespace Azimuth
{
    /// <summary>
    /// A collection of methods for extracting information about <see cref="Spline"/> types.
    /// </summary>
    public static class SplineHelper
    {



        public static float3 NormalFromTangent(float3 tangent, float3 up){
            return math.cross(tangent, up);
        }


        public static Spline OffsetSpline(Spline spline, float offset, int resolution = 1){
            int originalKnotsSize = spline.Count;
            int knotsSize = originalKnotsSize * resolution;
            Spline offsetSpline = new Spline(knotsSize, true);

            for(int i = 0; i < knotsSize; i++){

                float splinePos = (float)i / (float)resolution;
                float splineTime = spline.CurveToSplineT(splinePos);
                int originalSplineIndex = Mathf.FloorToInt(i / resolution);

                spline.Evaluate(splineTime, out var p_position, out var p_tangent, out var p_upVector);
                
                float3 deltaPosition = math.normalizesafe( NormalFromTangent(p_tangent,p_upVector) ) * offset;

                BezierKnot splineKnot = spline[originalSplineIndex];

                BezierKnot offsetKnot = new BezierKnot(p_position + deltaPosition , splineKnot.TangentIn, splineKnot.TangentOut, splineKnot.Rotation);

                TangentMode tangentMode = spline.GetTangentMode(originalSplineIndex);

                offsetSpline.Insert(i, offsetKnot, tangentMode);

            }

            return offsetSpline;
        }

        public static Spline RealSmoothSpline(Spline spline){
            int knotsSize = spline.Count;

            for(int i = 0; i < knotsSize; i++){

                int next = i + 1;
                if(next >= knotsSize){
                    next = 0;
                }
                int prev = i - 1;
                if(prev < 0){
                    prev = knotsSize - 1;
                }
                float3 node = spline[i].Position;
                float3 nodeNext = spline[next].Position;
                float3 nodePrev = spline[prev].Position;

                Vector3 nextDiff = Vector3.Normalize(nodeNext - node) ;
                Vector3 prevDiff = Vector3.Normalize(nodePrev - node) ;

                Vector3 siblingsMidPoint = (nextDiff + prevDiff) * 0.5f;

                float northNextAngle = Vector3.SignedAngle(nextDiff, Vector3.forward , Vector3.down);
                float northPrevAngle = Vector3.SignedAngle(prevDiff, Vector3.forward , Vector3.down);

                float signMulti = Mathf.Sign( northPrevAngle - northNextAngle );

                float northAngle = Vector3.SignedAngle(siblingsMidPoint, Vector3.forward , Vector3.down);
                float smoothAngle = northAngle - (signMulti * 90f);


/*
                //get angle at point between siblings
                float siblingsAngle = Vector3.SignedAngle(nextDiff, prevDiff, Vector3.up);
                float northAngle = Vector3.SignedAngle(nextDiff, Vector3.forward , Vector3.down); //gives correct polarity with down... rather than up..

                float smoothAngle =  northAngle + (siblingsAngle * 0.5f) - 90f;

                
*/

                Debug.Log( i + " northNextAngle= " + northNextAngle + " northPrevAngle= " + northPrevAngle + "  diff (" + (northPrevAngle - northNextAngle)  + ")" );
                Debug.Log( i + " northAngle= " + northAngle + " smoothAngle= " + smoothAngle );


                Quaternion rotation = Quaternion.Euler(0, smoothAngle, 0);

                //Debug.Log( i + " siblingsAngle= " + siblingsAngle + 
                

                //Debug.Log( "Rotation at " + i + " = " + ((Quaternion)spline[i].Rotation).eulerAngles );

                BezierKnot splineKnot = spline[i];
                splineKnot.Rotation = rotation;
                spline[i] = splineKnot;

            }

            return spline;


        }

        public static Spline ReduceByDistance(Spline spline, float mergeDistance){

            float sqrMergeDistance = mergeDistance * mergeDistance;
            //check in reverse order
            int i = spline.Count - 1;
            while( i >= 0){
                int next = i + 1;
                if(next >= spline.Count){
                    next = 0;
                }

                float3 node = spline[i].Position;
                float3 nodeNext = spline[next].Position;
                float sqrDistance = ((Vector3)(node - nodeNext)).sqrMagnitude;
                Debug.Log(i + " sqr distance " + sqrDistance);
                if( sqrDistance < sqrMergeDistance){
                    Debug.Log(" Removing " + i);
                    //move the next node to the median position
                    BezierKnot splineKnot = spline[next];
                    splineKnot.Position = (node + nodeNext) * 0.5f;
                    spline[next] = splineKnot;

                    spline.RemoveAt(i);

                    //reset the loop, start over
                    i = spline.Count - 1;

                }else{
                    i--;
                }
            }
            return spline;
        }

        public static Spline Spherise(Spline spline){
            int knotsSize = spline.Count;
            float weighting = 2f;
            float weightingDivisor = 1f / (weighting + 2f);

            Spline sphereSpline = new Spline(knotsSize, true);

            for(int i = 0; i < knotsSize; i++){

                                int next = i + 1;
                if(next >= knotsSize){
                    next = 0;
                }
                int prev = i - 1;
                if(prev < 0){
                    prev = knotsSize - 1;
                }

                float3 node = spline[i].Position;
                float3 nodeNext = spline[next].Position;
                float3 nodePrev = spline[prev].Position;

                float3 averagedPosition = ((node * weighting) + nodeNext + nodePrev) * weightingDivisor; //weighted average


                BezierKnot splineKnot = spline[i];

                splineKnot.Position = averagedPosition;

                TangentMode tangentMode = spline.GetTangentMode(i);

                sphereSpline.Insert(i, splineKnot, tangentMode);

            }

            return sphereSpline;

        }




        //Center spline knots around zero position 
        public static Spline CenterAroundZeroZero(Spline spline){
        //private Vector3[] CenterAroundZeroZero(Vector3[] randomisedPoints){
            int knotsSize = spline.Count;

            Bounds bounds = GetKnotBounds(spline);
/*
            Bounds bounds = new Bounds(spline[0].Position, Vector3.one);

            for (int i = 1; i < knotsSize; i++) {
                bounds.Encapsulate(spline[i].Position);
            }
*/
            //Debug.Log("Offset bounds : " + bounds.center);

            if( Mathf.Abs(bounds.center.x) > 1f || Mathf.Abs(bounds.center.y) > 1f){
                //offset
                float3 offset = bounds.center;
                offset.y = 0;

                for (int i = 0; i < knotsSize; i++) {

                    //randomisedPoints[i] -= offset;
                    BezierKnot splineKnot = spline[i];
                    splineKnot.Position -= offset;
                    spline[i] = splineKnot;

                }

            }

            return spline;

        }





        public static Spline ModifyTension(Spline spline, float tension){
            //tension = Mathf.Clamp01(tension);
            Debug.Log("Setting tension to " + tension);
            int knotsSize = spline.Count;

            spline.SetTangentMode(TangentMode.AutoSmooth);
            SplineRange range = new SplineRange(0, knotsSize);
            spline.SetAutoSmoothTension(range, tension);
            spline.Warmup();
            return spline;

        }


        /*
         * **** Spline Bound Helpers ******
        */

        public static Bounds GetKnotBounds(Spline spline){

            Bounds knotBounds = new Bounds();

            for(int j = 0; j < spline.Count; j++){
                knotBounds.Encapsulate(spline[j].Position);
            }

            return knotBounds;
        }



        //we generate bounding boxes for sections of the curves and then compare them to see if there are any overlaps, iterating down
        // generate an initial bounding per knot, if we din an over lap then we iterate down recursively
        // note the recusion loop level is maxed at 2 currently as this seems to be the fidelity we need
        public static bool CheckCurveOverlaps(Spline spline){
            bool splinechecksOkay = true;
            //need to get 0-1 positions of the knots ---> spline.CurveToSplineT(i); -- do this rewrite when you have time!
            float[] knotLengths = new float[spline.Count];
            float knotLengthTotal = 0;

            for(int i = 0; i < spline.Count; i++){
                knotLengths[i] = spline.GetCurveLength(i);
                knotLengthTotal += knotLengths[i];
            }

            Bounds[] splineSectionBounds = new Bounds[ knotLengths.Length ];
            float startPos = 0f;
            float knotPos = 0f;
            float gapsize = 0.001f;
            float[] startPosArray = new float[ knotLengths.Length ];
            float[] endPosArray = new float[ knotLengths.Length ];
            for(int i = 0; i < knotLengths.Length; i++){
                knotPos += ( knotLengths[i] / knotLengthTotal );
                startPosArray[i] = startPos+gapsize;
                endPosArray[i] = knotPos-gapsize;

                splineSectionBounds[i] = GetSplineSectionBounds( spline, startPosArray[i],  endPosArray[i], 3);
                startPos = knotPos;
            }

            for(int i = 0; i < splineSectionBounds.Length; i++){
                for(int j = 0; j < splineSectionBounds.Length; j++){
                    if(i != j){
                        if(splineSectionBounds[i].Intersects( splineSectionBounds[j]) ){
                            
                            bool isFineOverlap = FineSectionOverlapCheck( spline, startPosArray[i], endPosArray[i], startPosArray[j], endPosArray[j] , 0);

                            if( isFineOverlap ){
                            //    Debug.Log("Regenerating Overlapping Curve " + i + " with " + j);
                                splinechecksOkay = false;
                                return splinechecksOkay;
                            }
                        }
                    }

                }
                
            }

            return splinechecksOkay;
        }


        public static bool FineSectionOverlapCheck(Spline u_Spline, float startPosA, float endPosA, float startPosB, float endPosB, int loops ){

            int resolution = 3;
            loops++;

            Bounds[] a_sectionBounds = new Bounds[ resolution ];
            Bounds[] b_sectionBounds = new Bounds[ resolution ];

            float a_runningPos = startPosA;
            float a_increments = (endPosA - startPosA) / resolution;

            float a_pos = 0f;
            float a_gapsize = 0.000001f * a_increments;

            float[] a_startPosArray = new float[ resolution ];
            float[] a_endPosArray = new float[ resolution ];

            float b_runningPos = startPosB;
            float b_increments = (endPosB - startPosB) / resolution;

            float b_pos = 0f;
            float b_gapsize = 0.001f * b_increments;

            float[] b_startPosArray = new float[ resolution ];
            float[] b_endPosArray = new float[ resolution ];


            for(int i = 0; i < resolution; i++){

                a_startPosArray[i] = a_runningPos + a_gapsize;
                a_runningPos += a_increments;
                a_endPosArray[i] = a_runningPos - a_gapsize;
                a_sectionBounds[i] = GetSplineSectionBounds( u_Spline, a_startPosArray[i],  a_endPosArray[i], 3);

                b_startPosArray[i] = b_runningPos + b_gapsize;
                b_runningPos += b_increments;
                b_endPosArray[i] = b_runningPos - b_gapsize;
                b_sectionBounds[i] = GetSplineSectionBounds( u_Spline, b_startPosArray[i],  b_endPosArray[i], 3);

            }
            
            //bool isFineOverlap = false;

            for(int i = 0; i < resolution; i++){
                for(int j = 0; j < resolution; j++){
                    
                        if(a_sectionBounds[i].Intersects( b_sectionBounds[j]) ){

                            //Debug.Log("Fine Overlapping Curve (" + loops + ") " + a_startPosArray[i] + " with " + b_startPosArray[j]);
                            if( loops <= 2){
                                return FineSectionOverlapCheck( u_Spline, a_startPosArray[i], a_endPosArray[i], b_startPosArray[j], b_endPosArray[j], loops );
                            }else{
                                return true;
                            }

                        }

                }
                
            }

            return false;

        }

        public static Bounds GetSplineSectionBounds(Spline spline, float startoffset, float endoffset, int resolution){

            if (spline.Count < 1)
                return default;

            float sectionIncrements = (endoffset - startoffset) / resolution;
            Vector3 pos = spline.EvaluatePosition( startoffset );

            Bounds bounds = new Bounds(pos, Vector3.zero);

            for (int i = 1; i <= resolution; i++) {
                pos = spline.EvaluatePosition( startoffset + (sectionIncrements * i) );
                bounds.Encapsulate(pos);
            }

            return bounds;
        }

        public static Bounds GetSplineOuterBounds(Spline spline, int resolution){

            return GetSplineSectionBounds(spline, 0f, 1f, resolution);

        }


        //try using the Evryway solution from https://www.evryway.com/interior-rectangle/ & https://github.com/Evryway/lir
        public static Bounds GetSplineInnerBounds(Spline spline, int resolution){


            float increments = 1f / (float)resolution;
            Vector2[] posV2s = new Vector2[ resolution ];
            int j = 0;
            for (float i = 0f; i <= 1f; i += increments) {

                Vector3 pos = spline.EvaluatePosition(i);
                posV2s[j] = new Vector2(pos.x, pos.z);
                
                j++;

            }

            LIR.CalculateInteriorCells(posV2s, out float[] xs, out float[] ys, out int[,] cells); // (Vector2[] vs, out float[] xs, out float[] ys, out int[,] cells)
            LIR.CalculateLargestInteriorRectangle(xs, ys, cells, out Bound2D best); // (float[] xs, float[] ys, int[,] cells, out Bound2D best)

            //Debug.Log($"{best.centre.F3()}, {best.size.F3()}, {best.axis_a.F3()}");

            return new Bounds(new Vector3(best.centre.x, 0f, best.centre.y) , new Vector3(best.size.x, 0f, best.size.y)); //Bounds(Vector3 center, Vector3 size); 



        }


/*
        public static Bounds GetSplineInnerBounds(Spline spline, int resolution){
            //work outwards from the "center" point
            Bounds knotBounds = GetKnotBounds(spline);
            Bounds innerBounds = new Bounds();

            Vector3 minPositive = Vector3.positiveInfinity;
            Vector3 maxNegative = Vector3.negativeInfinity;

            float relativeMinDistancePxPz = Mathf.Infinity; //positive x , positive z
            float relativeMinDistanceNxPz = Mathf.Infinity;
            float relativeMinDistancePxNz = Mathf.Infinity;
            float relativeMinDistanceNxNz = Mathf.Infinity;

            Vector3[] relativeBoundPositions = new Vector3[4];

            //look for minimum points off the "center" point
            float increments = 1f / (float)resolution;
            for (float i = 0f; i <= 1f; i += increments) {

                Vector3 pos = spline.EvaluatePosition(i);
                Vector3 relative = pos - knotBounds.center;//relative position with respect to the center
                float r_distance = relative.magnitude;


                //Positive x Positive z
                if( relative.x >= 0 && relative.z >= 0){
                    if( r_distance < relativeMinDistancePxPz){
                        relativeMinDistancePxPz = r_distance;
                        relativeBoundPositions[0] = pos;
                    }
                //Negative x Positive z
                }else if( relative.x < 0 && relative.z >= 0 ){
                    if( r_distance < relativeMinDistanceNxPz){
                        relativeMinDistanceNxPz = r_distance;
                        relativeBoundPositions[1] = pos;
                    }

                //Positive x Negative z
                }else if( relative.x >= 0 && relative.z < 0 ){
                    if( r_distance < relativeMinDistancePxNz){
                        relativeMinDistancePxNz = r_distance;
                        relativeBoundPositions[2] = pos;
                    }

                // Negative x Negative z
                }else{
                    if( r_distance < relativeMinDistanceNxNz){
                        relativeMinDistanceNxNz = r_distance;
                        relativeBoundPositions[3] = pos;
                    }
                }


                
                //run through x y z 
                for(int v = 0;v < 3; v++){
                    if( relative[v] > 0){
                        if( relative[v] < minPositive[v]){
                            minPositive[v] = relative[v];
                        }
                    }else{
                        //less than zero
                        if( relative[v] > maxNegative[v]){
                            maxNegative[v] = relative[v];
                        }
                    }
                }
            }

            for(int j = 0; j<4; j++){
                if( relativeBoundPositions[j] != null){
                    Debug.Log("Min Bounds" + relativeBoundPositions[j]);
                    innerBounds.Encapsulate( relativeBoundPositions[j] );
                }

            }
            //innerBounds.Encapsulate( minPositive + knotBounds.center);
            //innerBounds.Encapsulate( maxNegative + knotBounds.center);

            return innerBounds;

        }
*/
        

    }
}
/**
Vector3 heading = prevPosition - p_position;// currentSplineData.position - nextSplineData.position;
                float t_angle = Vector3.SignedAngle(heading, Vector3.forward, Vector3.up);
                **/