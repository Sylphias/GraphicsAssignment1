#include "surf.h"
#include "extra.h"
#include "math.h"
using namespace std;

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }

	// Need to check if the end and the start of the curve are around the same
	inline bool approx(const Vector3f& lhs, const Vector3f& rhs)
	{
		const float eps = 1e-8f;
		return (lhs - rhs).absSquared() < eps;
	}
}



Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.
	float stepSize = (M_PI*2) / steps;
	float currRotate=0;
	unsigned counter = 0;
	for (int i = 0; i <= steps; i++) {
		for (int pointIdx = 0; pointIdx < profile.size(); pointIdx++) {
			Matrix3f rotation = Matrix3f::rotateY(currRotate);
			surface.VV.push_back(rotation*profile[pointIdx].V);

			//negate the normals
			Vector3f negatedNormals(profile[pointIdx].N);
			negatedNormals.negate();
			surface.VN.push_back(rotation* negatedNormals);


			if (i > 0 && pointIdx !=0) {
				surface.VF.push_back(Tup3u(counter, counter - 1, counter - profile.size()));
			}
			if (i < steps && pointIdx != profile.size() - 1) {
				surface.VF.push_back(Tup3u(counter, counter + 1, counter + profile.size()));
			}

			counter++;
		}
		currRotate += stepSize;
	}
 
    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;
    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

	// check if the curve forms a loop. 
	bool isLoop;
	if (isLoop = approx(sweep.back().V, sweep.front().V)) {
		cerr << "this curve is a loop" << endl;
	}
    // TODO: Here you should build the surface.  See surf.h for details.
	unsigned counter = 0;
	for (unsigned step = 0; step < sweep.size(); step++) {
		for (unsigned pstep = 0; pstep < profile.size(); pstep++) {
			surface.VV.push_back(sweep[step].V + sweep[step].B*profile[pstep].V.x() + sweep[step].N*profile[pstep].V.y());
			Vector3f negated(profile[pstep].N);
			negated.negate();
			Matrix3f bnt(sweep[step].B, sweep[step].N, sweep[step].V);
			surface.VN.push_back(bnt*negated);
			if (step > 0) {
				surface.VF.push_back(Tup3u(counter, counter - profile.size(), counter - 1));
			}
			if (step < sweep.size()-1) {
				surface.VF.push_back(Tup3u(counter, counter + profile.size(), counter + 1));
			}
			else if (pstep != profile.size() - 1 && isLoop) {
			
				//push final faces if it is a loop.
				surface.VF.push_back(Tup3u(counter, pstep, counter + 1));
				surface.VF.push_back(Tup3u(pstep, pstep + 1, counter + 1));
			
			}
			counter++;
		}
		
	}
	
    return surface;
}

void drawSurface(const Surface &surface, bool shaded)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {        
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        
        glColor4f(0.4f,0.4f,0.4f,1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

void drawNormals(const Surface &surface, float len)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1);
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i=0; i<surface.VV.size(); i++)
    {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (unsigned i=0; i<surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i=0; i<surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
