using UnityEngine;
// this is an old school racing line calculation by Remi Coulom, I'm sure it could be rewritten neatly using Vector2s.. one day I'll have time for that stuff!
// Orignal is from  https://rars.sourceforge.net/ https://rars.sourceforge.net/
// Then reused in TORCS https://torcs.sourceforge.net/ , https://en.wikipedia.org/wiki/TORCS and this version is ported form VDrift https://github.com/VDrift/vdrift/

namespace UnityRemiCoulom
{

	public class SmoothRacingLine
	{
		
		private float[] tx;
		private float[] ty;
		private float[] tRInverse;
		private float[] txLeft;
		private float[] tyLeft;
		private float[] txRight;
		private float[] tyRight;
		private float[] tLane;
		private int Divs;

		private static float SecurityR = 100.0f; // Security radius
		private static float SideDistExt = 2.0f; // Security distance wrt outside
		private static float  SideDistInt = 1.0f; // Security distance wrt inside
		private static int Iterations = 100; // Number of smoothing operations

		private static float Mag(float x, float y){
			return Mathf.Sqrt((x)*(x)+(y)*(y));
		}


	/*	
	#define Min(X,Y) ((X)<(Y)?(X):(Y)) ==>  Mathf.Min
	#define Max(X,Y) ((X)>(Y)?(X):(Y))  ==>  Mathf.Max
	*/

		public void CalcRaceLine()
		{
			int stepsize = 128;

			//abort if the track isn't long enough
			if (tx.Length < stepsize)
			{
				Debug.Log("Aborting, the track isn't long enough tx.Length: " + tx.Length + " stepsize: " + stepsize);
				return;
			}


			//TLaneDebug("Before Smoothing Loop");

			//
			// Smoothing loop
			//
			//for (int Step = stepsize; Step > 0; Step /= 2 )
			for (int Step = 128; (Step /= 2) > 0;)
			{
				for (int i = Iterations * (int)( Mathf.Sqrt( (float)Step )); --i >= 0; )
				{
					Smooth(Step);
					//TLaneDebug(i + " Smooth " + Step);
				}
				Interpolate(Step);
				//TLaneDebug("Interpolate " + Step);
			}

			//TLaneDebug("After Smoothing Loop");

	// May not need to do the below...	
			//
			// Compute curvature along the path
			//
			for (int i = Divs; --i >= 0;)
			{
				int next = (i + 1) % Divs;
				int prev = (i - 1 + Divs) % Divs;

				float rInverse = GetRInverse(prev, tx[i], ty[i], next);
				tRInverse[i] = rInverse;
			}

		}

		/////////////////////////////////////////////////////////////////////////////
		// Update tx and ty arrays
		/////////////////////////////////////////////////////////////////////////////
		private void UpdateTxTy(int i)
		{
			tx[i] = tLane[i] * txRight[i] + (1f - tLane[i]) * txLeft[i];
			ty[i] = tLane[i] * tyRight[i] + (1f - tLane[i]) * tyLeft[i];

		}

		/////////////////////////////////////////////////////////////////////////////
		// Compute the inverse of the radius
		/////////////////////////////////////////////////////////////////////////////
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



		/////////////////////////////////////////////////////////////////////////////
		// Smooth path
		/////////////////////////////////////////////////////////////////////////////

		private void Smooth(int Step)
		{
			
			int prev = ((Divs - Step) / Step) * Step; //Smooth 128 prev:0 prevprev:-128 next:128 nextnext:256
			int prevprev = (prev - Step);// % Divs ;  // (i + shift) % alphabet.Length
			
			int next = Step;
			int nextnext = (next + Step);// % Divs ;


			//Debug.Log("Smooth " + Step + " prev:" + prev + " prevprev:" + prevprev + " next:" + next + " nextnext:" + nextnext);

			// assert() : Expression to be evaluated. If this expression evaluates to 0, this causes an assertion failure that terminates the program
			Debug.Assert(prev >= 0);
			//std::cout << Divs << ", " << Step << ", " << prev << ", " << tx.size() << std::endl;
			Debug.Assert(prev < (int)tx.Length);
			Debug.Assert(prev < (int)ty.Length);
			Debug.Assert(next < (int)tx.Length);
			Debug.Assert(next < (int)ty.Length);

			for (int i = 0; i <= Divs - Step; i += Step)
			{
				float ri0 = GetRInverse(prevprev, tx[prev], ty[prev], i);
				float ri1 = GetRInverse(i, tx[next], ty[next], nextnext);
				float lPrev = Mag(tx[i] - tx[prev], ty[i] - ty[prev]);
				float lNext = Mag(tx[i] - tx[next], ty[i] - ty[next]);

				float TargetRInverse = (lNext * ri0 + lPrev * ri1) / (lNext + lPrev);

				float Security = lPrev * lNext / (8f * SecurityR);
				AdjustRadius(prev, i, next, TargetRInverse, Security);

				prevprev = prev;
				prev = i;
				next = nextnext;
				nextnext = next + Step;
				if (nextnext > Divs - Step)
				{
					nextnext = 0;
				}
			}
		}


		/////////////////////////////////////////////////////////////////////////////
		// Interpolate between two control points
		/////////////////////////////////////////////////////////////////////////////
		private void StepInterpolate(int iMin, int iMax, int Step)
		{
			int next = (iMax + Step) % Divs;
			if (next > Divs - Step)
				next = 0;

			int prev = (((Divs + iMin - Step) % Divs) / Step) * Step;
			if (prev > Divs - Step)
				prev -= Step;

			float ir0 = GetRInverse(prev, tx[iMin], ty[iMin], iMax % Divs);
			float ir1 = GetRInverse(iMin, tx[iMax % Divs], ty[iMax % Divs], next);
			for (int k = iMax; --k > iMin;)
			{
				float x = (float)(k - iMin) / (float)(iMax - iMin);
				float TargetRInverse = x * ir1 + (1f - x) * ir0;
				AdjustRadius(iMin, k, iMax % Divs, TargetRInverse);
			}
		}

		/////////////////////////////////////////////////////////////////////////////
		// Calls to StepInterpolate for the full path
		/////////////////////////////////////////////////////////////////////////////
		private void Interpolate(int Step)
		{
			if (Step > 1)
			{
				int i;
				for (i = Step; i <= Divs - Step; i += Step){
					StepInterpolate(i - Step, i, Step);
				}
				StepInterpolate(i - Step, Divs, Step);
			}
		}


		/////////////////////////////////////////////////////////////////////////////
		// Change lane value to reach a given radius
		/////////////////////////////////////////////////////////////////////////////
		private void AdjustRadius(int prev, int i, int next, float TargetRInverse, float Security = 0f)
		{
			float OldLane = tLane[i];

			float Width = Mag((txLeft[i]-txRight[i]),(tyLeft[i]-tyRight[i]));

			//
			// Start by aligning points for a reasonable initial lane
			//
			tLane[i] =  (-(ty[next] - ty[prev]) * (txLeft[i] - tx[prev]) +
						  (tx[next] - tx[prev]) * (tyLeft[i] - ty[prev])) /
						( (ty[next] - ty[prev]) * (txRight[i] - txLeft[i]) -
						  (tx[next] - tx[prev]) * (tyRight[i] - tyLeft[i]));


			if(i == 0){
				Debug.Log("next:"+next+ "prev:"+ prev + " " + 
				          (ty[next] - ty[prev]) + " * " + (txLeft[i] - tx[prev]) + " + " +
						  (tx[next] - tx[prev]) + " * " + (tyLeft[i] - ty[prev]) + ") / (" +
						  (ty[next] - ty[prev]) + " * " + (txRight[i] - txLeft[i]) + " - " + 
						  (tx[next] - tx[prev]) + " * " + (tyRight[i] - tyLeft[i]) );
			}
			// TLaneDebug("AdjustRadius aligning ");

			// the original algorithm allows going outside the track
			if( float.IsNaN(tLane[i]) ){
				tLane[i] = 0.5f;
			}
			else if (tLane[i] < -0.2f)
			{
				tLane[i] = -0.2f;
			}
			else if (tLane[i] > 1.2f)
			{
				tLane[i] = 1.2f;
			}
			
			/*
			if (tLane[i] < 0.0f)
			{
				tLane[i] = 0.0f;
			
			}
			else if (tLane[i] > 1.0f)
			{
				tLane[i] = 1.0f;
			}
			*/
			UpdateTxTy(i);

			//
			// Newton-like resolution method
			//
			const float dLane = 0.0001f;

			float dx = dLane * (txRight[i] - txLeft[i]);
			float dy = dLane * (tyRight[i] - tyLeft[i]);

			float dRInverse = GetRInverse(prev, tx[i] + dx, ty[i] + dy, next);

			if(i == 0) TLaneDebug(" Before dRInverse ");

			if (dRInverse > 0.000000001f)
			{
				tLane[i] += (dLane / dRInverse) * TargetRInverse;

				float ExtLane = (SideDistExt + Security) / Width;
				float IntLane = (SideDistInt + Security) / Width;
				if (ExtLane > 0.5f)
					ExtLane = 0.5f;
				if (IntLane > 0.5f)
					IntLane = 0.5f;

				if (TargetRInverse >= 0.0f)
				{
					if (tLane[i] < IntLane)
						tLane[i] = IntLane;
					if (1f - tLane[i] < ExtLane)
					{
						if (1f - OldLane < ExtLane)
							tLane[i] = Mathf.Min(OldLane, tLane[i]);
						else
							tLane[i] = 1f - ExtLane;
					}
				}
				else
				{
					if (tLane[i] < ExtLane)
					{
						if (OldLane < ExtLane)
							tLane[i] = Mathf.Max(OldLane, tLane[i]);
						else
							tLane[i] = ExtLane;
					}
					if (1f - tLane[i] < IntLane)
						tLane[i] = 1f - IntLane;
				}
			}

			if(i == 0) TLaneDebug(" After dRInverse ");

			UpdateTxTy(i);
		}


		//public void LoadData(const RoadStrip & road)

		//acess the computed positions
		public Vector3[] BAD_GetRacingLineVector3(){
			Vector3[] V3Points = new Vector3[Divs];
			for (int i = 0; i < Divs; i++)
			{
				V3Points[i] = new Vector3(tx[i], 0f, ty[i]);
			}
			return V3Points;
		}

		private void TLaneDebug(string str = ""){
			Debug.Log("=== " + str + " TLane  0:" + tLane[0] + " " +tLane.Length+":" + tLane[tLane.Length - 1] );

			//Debug.Log(i + " T:" + tLane[i]);
			/*
			for( int i = 0; i < tLane.Length; i++ ){
				Debug.Log(i + " T:" + tLane[i]);
			}*/
		}


		public Vector3[] GetRacingLineVector3(){

			Vector3[] V3Points = new Vector3[Divs];

			//TLaneDebug("GetRacingLineVector3");
			//int count = 0;
			for (int i = 0; i < Divs; i++)
			{
				Vector3 leftPoint = new Vector3(txLeft[i], 0f, tyLeft[i]);
				Vector3 rightPoint = new Vector3(txRight[i], 0f, tyRight[i]);

				Debug.Log(i + " L:" + leftPoint.ToString("F1") + " L:" + rightPoint.ToString("F1") + " T:" + tLane[i]);

				V3Points[i] = (leftPoint * (1.0f - tLane[i])) + (rightPoint * (tLane[i])) ;
				//int count++;


				//V3Points[i] = new Vector3(tx[i], 0f, ty[i]);
			}
			return V3Points;
		}

/**
			int count = 0;

	for (auto & p : patchlist)
	{
//// GetPoint -- returns points[x][y]
///access corners of the patch (front left, front right, back left, back right)
///	const Vec3 & GetFL() const {return points[0][0];}
///	const Vec3 & GetFR() const {return points[0][3];}
///	const Vec3 & GetBL() const {return points[3][0];}
///	const Vec3 & GetBR() const {return points[3][3];}

		// GetBL & GetBR
		auto point = p.GetPoint(3,0)*(1.0-tLane[count]) + p.GetPoint(3,3)*(tLane[count]);
		p.SetRacingLine(point, tRInverse[count]);
		//std::cout << point << std::endl;
		count++;
	}


void SetRacingLine(const Vec3 & position, float curvature)
	{
		racing_line = position;
		track_curvature = curvature;
		have_racingline = true;
	}


	**/
		/** sending in 
				m_txLeft[k] = outerPos.x;
                m_tyLeft[k] = outerPos.z;
                m_txRight[k] = innerPos.x;
                m_tyRight[k] = innerPos.z;
		**/
		public void SetPerimetersXYData(float[] m_txLeft, float[] m_tyLeft, float[] m_txRight, float[] m_tyRight)
		{

			Divs = m_txLeft.Length; //all these these should always be the same Lengths, lets use the first!

			txLeft = m_txLeft;
			tyLeft = m_tyLeft;
			txRight = m_txRight;
			tyRight = m_tyRight;
/*
			txRight = m_txLeft;
			tyRight = m_tyLeft;
			txLeft = m_txRight;
			tyLeft = m_tyRight;
*/
			tLane = new float[Divs];
			tx = new float[Divs];
			ty = new float[Divs];
			tRInverse = new float[Divs];

			//int count = 0;
			for (int i = 0; i < Divs; i++)
			{
				tLane[i] = 0.5f; //sets the lane to the midpoint between Left and Right positions
				//tx[i] = 0.0f;
				//ty[i] = 0.0f;
				tx[i] = 0.5f * (txRight[i] + txLeft[i]);
				ty[i] = 0.5f * (tyRight[i] + tyLeft[i]);

				tRInverse[i] = 0.0f;

				//UpdateTxTy(i);
				//count++;

			}

			TLaneDebug("Init SetPerimetersXYData");




	/*
			tx.clear();
			ty.clear();
			tRInverse.clear();
			txLeft.clear();
			tyLeft.clear();
			txRight.clear();
			tyRight.clear();
			tLane.clear();

			const std::vector<RoadPatch> & patchlist = road.GetPatches();
			Divs = patchlist.size();

			tx.reserve(Divs);
			ty.reserve(Divs);
			tRInverse.reserve(Divs);
			txLeft.reserve(Divs);
			tyLeft.reserve(Divs);
			txRight.reserve(Divs);
			tyRight.reserve(Divs);
			tLane.reserve(Divs);

			int count = 0;
			for (const auto & p : patchlist)
			{
				txLeft.push_back(p.GetPoint(3,0)[1]);
				tyLeft.push_back(-p.GetPoint(3,0)[0]);
				txRight.push_back(p.GetPoint(3,3)[1]);
				tyRight.push_back(-p.GetPoint(3,3)[0]);
				tLane.push_back(0.5);
				tx.push_back(0.0);
				ty.push_back(0.0);
				tRInverse.push_back(0.0);
				UpdateTxTy(count);

				count++;
			}
	*/
		}

	}
}