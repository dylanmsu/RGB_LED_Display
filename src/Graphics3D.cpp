#include "Graphics3D.h"

Graphics3D::Graphics3D(MatrixPanel *matrixPanel)
{
    this->matrixPanel = matrixPanel;
}

Graphics3D::~Graphics3D()
{
}

double *rotate(double *v, int len, double rX, double rY, double rZ)
{
    double cosa = cos(rZ);
    double sina = sin(rZ);

    double cosb = cos(rY);
    double sinb = sin(rY);

    double cosc = cos(rX);
    double sinc = sin(rX);

    double Axx = cosa*cosb;
    double Axy = cosa*sinb*sinc - sina*cosc;
    double Axz = cosa*sinb*cosc + sina*sinc;

    double Ayx = sina*cosb;
    double Ayy = sina*sinb*sinc + cosa*cosc;
    double Ayz = sina*sinb*cosc - cosa*sinc;

    double Azx = -sinb;
    double Azy = cosb*sinc;
    double Azz = cosb*cosc;

    // apply matrix transformation to every vertex
    for (int i=0;i<len*3;i+=3)
    {
        double px = v[i+0];
        double py = v[i+1];
        double pz = v[i+2];

        v[i+0] = Axx*px + Axy*py + Axz*pz;
        v[i+1] = Ayx*px + Ayy*py + Ayz*pz;
        v[i+2] = Azx*px + Azy*py + Azz*pz;
    }	
    return v;
}

//sorts an array and returns an array of the indices
void sort(float *arr,const int n, int *idx){
	int i;
	int j;
	int tmp;
	for (i=0;i<n;i++){
		idx[i] = i;
	}
	for (i=0;i<n;i++){
		for (j=i+1;j<n;j++){
			if (arr[idx[i]] < arr[idx[j]]){
				tmp = idx[i];
				idx[i] = idx[j];
				idx[j] = tmp;
			}
		}
	}
}

float clamp(float num, float mini, float maxi){
	return min(max(maxi,num),mini);
}
float max(float a, float b){
	if (a<b)return b;
	else return a;
}
float min(float a, float b){
	if (a>b)return b;
	else return a;
}

//gets all the Z values of the faces
void Zarr(double *verts, int *faces, int len, float *arr){
	for (int i=0;i<len;i++){
		float c = 0;
		for (int j=0;j<4;j++){
			c += verts[faces[i*4+j]*3+1];
		}
		c /= 4;
		arr[i] = c;
	}
}

//devides the vector by its length
void normalize(float *vec){
	float inv_mag = 1.0 / sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
	vec[0] *= inv_mag;
	vec[1] *= inv_mag;
	vec[2] *= inv_mag;
}

//calculates the cross product of two vectors
void cross(const float *a, const float *b, float *out){
	out[0] = a[1]*b[2] - a[2]*b[1];
	out[1] = a[2]*b[0] - a[0]*b[2];
	out[2] = a[0]*b[1] - a[1]*b[0];
}

//gets the normal vector of a face
void getNormal(const double *verts, const int *faces, const int len, const int index, float *out){
	float edgeA[3] = {0,0,0};
	float edgeB[3] = {0,0,0};
	for (int i=0;i<3;i++){ 
		edgeA[i] = verts[faces[index*4+1]*3+i] - verts[faces[index*4+0]*3+i];
		edgeB[i] = verts[faces[index*4+2]*3+i] - verts[faces[index*4+0]*3+i];
	}
	cross(edgeA,edgeB,out);
	normalize(out);
}

//calculates the dot product
float dot(const float *a, const float *b){
	return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

//gets the reflected vector
//https://www.opengl.org/discussion_boards/showthread.php/155886-Reflection-vector
void reflect(float d[3], float n[3], float reflected[3]){
        float dot_d_n = dot(d,n);
        reflected[0] = 2*dot_d_n*n[0]-d[0];
        reflected[1] = 2*dot_d_n*n[1]-d[1];
        reflected[2] = 2*dot_d_n*n[2]-d[2];
}

float pyth(float vec1[3], float vec2[3]){
    float x = vec1[0]-vec2[0];
    float y = vec1[1]-vec2[1];
    float z = vec1[2]-vec2[2];
    return x*x+y*y+z*z;
}

void faceCenter(double *verts, int *faces, int k, float center[3]){
    for(int i=0; i<4; i++){
        int vert = faces[k*4 + i];
        center[0] += verts[vert*3 + 0];
        center[1] += verts[vert*3 + 1];
        center[2] += verts[vert*3 + 2];
    }
    center[0] /= 4;
    center[1] /= 4;
    center[2] /= 4;
}

//void drawSolid(double *verts, int *faces, int facelen, float r, float g, float b, uint8_t **buffer){
void Graphics3D::drawSolid(double *verts, int *faces, float *colors, int facelen, double zoom){
    int verts_per_face = 4;
	const int cx = 32/2;
    const int cy = 32/2;
    
    int buf[facelen] = {0};
    float Zarray[facelen] = {0};
    
    Zarr(verts,faces,facelen,Zarray); //gets array of all the z positions of the faces and dump them in "Zarray"
	sort(Zarray,facelen,buf);        //get sorted indices of the z array to apply the "painter’s algorithm"
    
    for (int j=0;j<facelen;j++){
		
		int i = buf[j];
        float cameraPos[3] = {0, 4, 0};    //position of the camera

        float x[verts_per_face] = {0};
        float y = 0;
        float z[verts_per_face] = {0};

        float px[verts_per_face] = {0};
        float py[verts_per_face] = {0};

        // here is where the 3d coordinates are mapped onto a 2d  xz surface
        for (int k=0; k<verts_per_face; k++) {
            x[k] =  verts[faces[i*verts_per_face+k]*3+0]+cameraPos[0];
            y =     verts[faces[i*verts_per_face+k]*3+1]+cameraPos[1];
            z[k] =  verts[faces[i*verts_per_face+k]*3+2]+cameraPos[2];

            x[k] *= zoom/y;
            z[k] *= zoom/y;

            px[k] = cx+x[k];
            py[k] = cy+z[k];
        }

        // the following code is to provide shaders
        //http://www.opengl-tutorial.org/beginners-tutorials/tutorial-8-basic-shading/
		
        // diffuse color of the 3d face
		float diffuseColor[3] = {colors[i*3 + 0], colors[i*3 + 1], colors[i*3 + 2]};

        // color of the licht source
		float lightColor[3] = {1,1,1};

        // color of the ambient licht
		float ambientColor[3] = {colors[i*3 + 0]/10, colors[i*3 + 1]/10, colors[i*3 + 2]/10};

		float lightPower = 10;
		float sun[3] = {-2,2,2}; //light source position
        float shininess = 5;
        float specularColor[3] = {1,1,1};

        // common
        float faceNormal[3] = {0};
        float sunNormal[3] = {sun[0], sun[1], sun[2]};
        float dist_sq = 0;
        if (enable_diffuse || enable_specular) {
            float centerFace[3] = {0};

            getNormal(verts, faces, facelen, i, faceNormal);
            normalize(sunNormal);
            faceCenter(verts, faces, i, centerFace);
            dist_sq = pyth(sun, centerFace);
        }

        // diffuse
        float cosTheta = 0;
        if (enable_specular) {
            cosTheta = clamp(dot(sunNormal,faceNormal),1,0);
        }

        // specular
        
        float cosAlpha = 0;
        if (enable_specular) {
            float reflection[3] = {0};
            float lightRay[3] = {-sunNormal[0], -sunNormal[1], -sunNormal[2]};
            float camNormal[3] = {cameraPos[0], cameraPos[1], cameraPos[2]};

            reflect(lightRay, faceNormal, reflection);
            normalize(camNormal);
            cosAlpha = clamp(dot(camNormal,reflection),1,0);
        }

        float color[3] = {0};
        for (int c=0; c<3; c++) {
            float ambient_component = ambientColor[c];
            float diffuse_component = 0;
            float specular_component = 0;

            if (enable_diffuse) {
                diffuse_component = diffuseColor[c]*lightColor[c]*lightPower*cosTheta/(dist_sq);
            }
            if (enable_specular) {
                specular_component = specularColor[c]*lightColor[c]*lightPower*pow(cosAlpha,shininess)/(dist_sq);
            }
            
            color[c] = clamp(ambient_component + diffuse_component + specular_component, 1, 0);
        }

        matrixPanel->fillQuat(px, py, color[0]*255, color[1]*255, color[2]*255);
    }
}

void Graphics3D::pushVertex(float x, float y, float z) {
    vertices.push_back(x);
    vertices.push_back(y);
    vertices.push_back(z);
}

void Graphics3D::pushQuat(int p1, int p2, int p3, int p4, uint8_t r, uint8_t g, uint8_t b) {
    faces.push_back(p1);
    faces.push_back(p2);
    faces.push_back(p3);
    faces.push_back(p4);

    face_colors.push_back(clamp(r/255.0,1,0));
    face_colors.push_back(clamp(g/255.0,1,0));
    face_colors.push_back(clamp(b/255.0,1,0));
}

void Graphics3D::setRotation(float x, float y, float z) {
    rotation[0] = x;
    rotation[1] = y;
    rotation[2] = z;
}

void Graphics3D::drawMesh() {
    double verts[vertices.size()];
    std::copy(vertices.begin(), vertices.end(), verts);

    int quats[faces.size()];
    std::copy(faces.begin(), faces.end(), quats);

    float colors[face_colors.size()];
    std::copy(face_colors.begin(), face_colors.end(), colors);

    drawSolid(rotate(verts,vertices.size()/3,rotation[0],rotation[1],rotation[2]),quats,colors,faces.size()/4,32);
}
