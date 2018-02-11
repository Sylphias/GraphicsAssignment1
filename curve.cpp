#include "curve.h"
#include "extra.h"
#include <cmath>
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
using namespace std;

namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }

	Matrix4f bezSplineBasis(1, -3, 3, -1,
		0, 3, -6, 3,
		0, 0, 3, -3,
		0, 0, 0, 1);

	Matrix4f bezSplineBasisDiff(-3, 6, -3, 0,
		3, -12, 9, 0,
		0, 6, -9, 0,
		0, 0, 3, 0);

	Matrix4f bSplineBasis(1.0f / 6, -3.0f / 6, 3.0f / 6, -1.0f / 6,
							4.0f / 6, 0, -6.0f / 6, 3.0f / 6,
							1.0f / 6, 3.0f / 6, 3.0f / 6, -3.0f / 6,
							0, 0, 0, 1.0f / 6);
    
}





Curve evalBezier( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
        exit( 0 );
    }

	if (steps == 0) {
		cerr << "Number of Steps cannot be 0" << endl;
		exit(0);
	}

    // TODO:
    // You should implement this function so that it returns a Curve
    // (e.g., a vector< CurvePoint >).  The variable "steps" tells you
    // the number of points to generate on each piece of the spline.
    // At least, that's how the sample solution is implemented and how
    // the SWP files are written.  But you are free to interpret this
    // variable however you want, so long as you can control the
    // "resolution" of the discretized spline curve with it.
	// Make sure that this function computes all the appropriate
	// Vector3fs for each CurvePoint: V,T,N,B.
	// [NBT] should be unit and orthogonal.

	// Also note that you may assume that all Bezier curves that you
	// receive have G1 continuity.  Otherwise, the TNB will not be
	// be defined at points where this does not hold.

	//populate matrix for ease of matrix operations
	// for each 4 control points, push in a matrix4f into a vector
	vector<Matrix4f>ctrlPts;
	for (unsigned a = 0; a < P.size() / 3; a++) {
		unsigned offset = a * 3;
		int loopBack = 0;
		//deal with loops, set start = end if they are approx the same.
		if (approx(P[0], P.back()) && a == P.size()-1) {
			loopBack = -1 * (offset + 3);
		}
		Matrix4f geo(P[(0+offset)].x(), P[1+offset].x(), P[2+offset].x(), P[3+offset+loopBack].x(),
					 P[(0+offset)].y(), P[1+offset].y(), P[2+offset].y(), P[3+offset+loopBack].y(),
					 P[(0+offset)].z(), P[1+offset].z(), P[2+offset].z(), P[3+offset+loopBack].z(),
					 0, 0, 0, 0);
		ctrlPts.push_back(geo);

	}



	Curve c;
	float stepValue = 1.0f / steps;

	// We begin drawing out each section of the bezier curve
	Vector3f oldBinormal;
	for (unsigned sectionNum = 0; sectionNum < ctrlPts.size(); sectionNum++) {
		float t = 0;
		// for each section we loop through each step to create the curve

		Vector3f binormal;
		if(sectionNum==0){

			// For loops, we need to set the arbVect to the binormal of the last value... ask if its really efficient to recalc the whole thing?

			// After quite awhile of thinking, I guess it's unavoidable to interpolate the difference in angle...
			Vector3f arbVect(0, 0, 1);
			oldBinormal = arbVect.normalized();
		}

		for (unsigned i = 0; i <= steps; i++) {
			// calculate q

			Vector4f powerBasis(1, t, pow(t, 2), pow(t, 3));
			Vector4f bt(ctrlPts[sectionNum]*bezSplineBasis*powerBasis);
			Vector4f btPrime(ctrlPts[sectionNum] *bezSplineBasisDiff*powerBasis);
			Vector3f Q = bt.xyz();
			Vector3f QPrime = btPrime.xyz();
			Vector3f tangent = QPrime.normalized();

			Vector3f normal = Vector3f::cross(oldBinormal, tangent).normalized();
			binormal = Vector3f::cross(tangent, normal).normalized();
			oldBinormal = binormal;
			CurvePoint n;
			n.V = Q;
			n.T = tangent;
			n.N = normal;
			n.B = binormal ;
			c.push_back(n);
			t += stepValue;

		}

		
	}

	// Find the difference in angle between the last and the first normal if the curve is a loop and the normals are different
	if (approx(c.back().V, c.front().V) && !approx(c.back().N,c.front().N)) {
		

		float angle = acos( Vector3f::dot( c.front().N, c.back().N));
		
		// rotate every point after the start by a fixed step
		float angleStep = angle / (c.size() - 1);
		float currAngle = angleStep;
		for (unsigned point = 1; point < c.size(); point++) {
			//rotate the normal and binormal wrt to tangent as axis
			c[point].B = Matrix3f::rotation(-1*c[point].T, currAngle)*c[point].B ;
			c[point].N = Matrix3f::rotation(-1 * c[point].T, currAngle)*c[point].N;
			currAngle += angleStep;
		}
	}

    cerr << "\t>>> evalBezier has been called with the following input:" << endl;

    cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;

    return c;
}

Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{
    // Check
    if( P.size() < 4 )
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit( 0 );
    }

    // TODO:
    // It is suggested that you implement this function by changing
    // basis from B-spline to Bezier.  That way, you can just call
    // your evalBezier function.
	// Basically reuse evalBezier to draw the curve.
	
	// Breakdown bspline into segments of 4 to do basis conversion
	bool max = false;
	vector<Vector3f> bezierPoints;
	for (unsigned i = 0; i < P.size() ; i++) {
		if (max) {
			break;
		}
		Matrix4f bsplineSeg;
		for (unsigned j = 0; j < 4; j++) {
			int offsetted = i + j;// segmentize using offsets of up to 4
			if (offsetted >= P.size()-1) {
				max = true;
			}
			Vector4f vec(P[offsetted].xyz(), 0);
			bsplineSeg.setCol(j, vec);
		}
		Matrix4f changedBasis(bsplineSeg*bSplineBasis*(bezSplineBasis.inverse()));
		int startingPt = 1;
		if (i == 0) {
			startingPt = 0;
		}
		for (unsigned j = startingPt; j < 4; j++) {
			bezierPoints.push_back(changedBasis.getCol(j).xyz());
		}
		
	}
	

    cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

    cerr << "\t>>> Control points (type vector< Vector3f >): "<< endl;
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t>>> " << P[i] << endl;
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;

    // Return an empty curve right now.
    return  evalBezier(bezierPoints, steps);;
}

Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).
    
    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }
    
    // Pop state
    glPopAttrib();
}

