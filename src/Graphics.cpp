#include "Graphics.h"

Graphics::Graphics(MatrixPanel *mat) :matrix(mat)
{
}

Graphics::~Graphics()
{
}

double k = 0;
double zoom = 32;
double freq = 0.1;    //color changing frequency
double camDist = 4.0; //distance the camera is to the wireframe
double pi = 3.14159265;
double rad = 2*pi;

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
	float x = vec[0];
	float y = vec[1];
	float z = vec[2];
	float len = sqrt(x*x+y*y+z*z);
	vec[0] = x/len;
	vec[1] = y/len;
	vec[2] = z/len;
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

//multiplies a vactor
void mul(float *vec, float value){
	vec[0]*=value;
	vec[1]*=value;
	vec[2]*=value;
}

void torus(double *verts, int *faces){
	int stacks = 5;
	int slices = 5;
	float radius = 0.5;
	float rad = pi*2;
	  
	for (int i=0; i<slices;i++){
		for (int j=0; j<stacks;j++){
			int indx = slices*i+j;
			float r = 1-radius*sin((rad/stacks)*j);
			verts[indx*3+0] = r*sin((rad/slices)*i);
			verts[indx*3+1] = radius*cos((rad/stacks)*j);
			verts[indx*3+2] = r*cos((rad/slices)*i);
			
			faces[indx*4+0] = (j+stacks+(stacks*i))%(stacks*slices);
			faces[indx*4+1] = ((((j+1)%stacks)+stacks)+(stacks*i))%(stacks*slices);
			faces[indx*4+2] = (((j+1)%stacks)+stacks*i)%(stacks*slices);
			faces[indx*4+3] = j+i*stacks;
		}
	}
}

//void drawSolid(double *verts, int *faces, int facelen, float r, float g, float b, uint8_t **buffer){
void Graphics::drawSolid(double *verts, int *faces, int facelen, float r, float g, float b){
	const int cx = 32/2;
    const int cy = 32/2; 
    
    int buf[facelen] = {0};        //initialise empty arrays
    float Zarray[facelen] = {0};   //
    
    Zarr(verts,faces,facelen,Zarray); //gets array of all the z positions of the faces and dump them in "Zarray"
	sort(Zarray,facelen,buf);        //get sorted indices of the z array
    
    for (int j=3;j<facelen;j++){//starts at 3 because we don't want the first 3 faces to be drawn
		
		int i = buf[j]*4;
        float dist = 4;    //distance of the camera

        float x[4] = {0,0,0,0};
        float y = 0;
        float z[4] = {0,0,0,0};

        for (int k=0; k<4; k++) {
            x[k] = verts[faces[i+k]*3+0];
            y = verts[faces[i+k]*3+1]+dist;
            z[k] = verts[faces[i+k]*3+2];
            x[k] *= zoom/y;
            z[k] *= zoom/y;
        }
		
		float diffuseColor[3] = {r,g,b};
		float lightColor[3] = {1,1,1};
		float ambientColor[3] = {r/10,g/10,b/10};
		float lightPower = 1;
		float sun[3] = {-1,1,1}; //light source position

		float nor[3] = {};
		normalize(sun);
		mul(sun,1);
		getNormal(verts, faces, facelen, i/4, nor);
		float dotp = dot(sun,nor);
		float dif = clamp(dotp,1,0);
		
		float red = ambientColor[0]+diffuseColor[0]*lightColor[0]*lightPower*dif/(sun[0]*sun[0]+sun[1]*sun[1]+sun[2]*sun[2]);
		float grn = ambientColor[1]+diffuseColor[1]*lightColor[1]*lightPower*dif/(sun[0]*sun[0]+sun[1]*sun[1]+sun[2]*sun[2]);
		float blu = ambientColor[2]+diffuseColor[2]*lightColor[2]*lightPower*dif/(sun[0]*sun[0]+sun[1]*sun[1]+sun[2]*sun[2]);
      
        float px[4] = {cx+x[0],cx+x[1],cx+x[2],cx+x[3]};
        float py[4] = {cy+z[0],cx+z[1],cx+z[2],cx+z[3]};
        matrix->fillPoly(px, py, 4, red*255, grn*255, blu*255);
  }
}

void Graphics::drawMesh(double verts[], uint16_t edges[],int edgeCnt, int r, int g, int b)
{
    // get center of the matrix
    const int cx = 32/2;
    const int cy = 32/2; 

    // for every edge
    for (int i=0;i<edgeCnt*2;i+=2){

        double xpoints[2] = {};
        double zpoints[2] = {};
        for (int j=0; j<2; j++) {
            float x = verts[edges[i+j]*3+0];
            float y = verts[edges[i+j]*3+1]+camDist;
            float z = verts[edges[i+j]*3+2];
            float f = zoom/y;
            xpoints[j] = x*f;
            zpoints[j] = z*f;
        }

        matrix->drawLine(cx+xpoints[0], cy+zpoints[0], cx+xpoints[1], cy+zpoints[1], r, g, b);
    }
}