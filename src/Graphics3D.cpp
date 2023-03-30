#include "Graphics3D.h"

Graphics3D::Graphics3D(MatrixPanel *matrixPanel)
{
    this->matrixPanel = matrixPanel;
}

Graphics3D::~Graphics3D()
{
    free(faces);
    free(vertices);
    free(face_colors);
    free(face_normals);
    free(transformed_vertices);
    free(transformed_face_normals);
}

void apply_matrix(const float *source_verts, float *dest_verts, const float *matrix, int len) {
    // for every vertex
    for (int i=0;i<len*3;i+=3)
    {
        float px = source_verts[i+0];
        float py = source_verts[i+1];
        float pz = source_verts[i+2];

        dest_verts[i+0] = matrix[0]*px + matrix[1]*py + matrix[2]*pz;
        dest_verts[i+1] = matrix[3]*px + matrix[4]*py + matrix[5]*pz;
        dest_verts[i+2] = matrix[6]*px + matrix[7]*py + matrix[8]*pz;
    }
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
void Zarr(float *verts, int *faces, int len, float *arr){
	for (int i=0;i<len;i++){
		float c = 0;
		for (int j=0;j<VERTS_PER_FACE;j++){
			c += verts[faces[i*VERTS_PER_FACE+j]*3+1];
		}
		c /= VERTS_PER_FACE;
		arr[i] = c;
	}
}

//devides the vector by its length
void normalize(float *vec){
    float inv_mag = inverse_rsqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
	//float inv_mag = 1.0 / sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
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
void getFaceNormal(const float *verts, const int *faces, const int index, float *out){
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
        float dot_d_n_2 = dot(d,n)*2;
        reflected[0] = dot_d_n_2*n[0]-d[0];
        reflected[1] = dot_d_n_2*n[1]-d[1];
        reflected[2] = dot_d_n_2*n[2]-d[2];
}

float dist_squared(float vec1[3], float vec2[3]){
    float x = vec1[0]-vec2[0];
    float y = vec1[1]-vec2[1];
    float z = vec1[2]-vec2[2];
    return x*x+y*y+z*z;
}

void face_center(float *verts, int *faces, int k, float center[3]){
    for(int i=0; i<VERTS_PER_FACE; i++){
        int vert = faces[k*VERTS_PER_FACE + i];
        center[0] += verts[vert*3 + 0];
        center[1] += verts[vert*3 + 1];
        center[2] += verts[vert*3 + 2];
    }
    center[0] /= VERTS_PER_FACE;
    center[1] /= VERTS_PER_FACE;
    center[2] /= VERTS_PER_FACE;
}

// precalculate face normals for performance gain
void Graphics3D::calculateNormals() {
    // allocate memory to store the normals
    face_normals = (float *) realloc(face_normals, sizeof(float *)*(face_count)*3);
    transformed_face_normals = (float *) realloc(transformed_face_normals, sizeof(float *)*(face_count)*3);

    for (int i=0; i<face_count; i++) {
        float faceNormal[3] = {0};
        
        getFaceNormal(vertices, faces, i, faceNormal);

        face_normals[i*3 + 0] = faceNormal[0];
        face_normals[i*3 + 1] = faceNormal[1];
        face_normals[i*3 + 2] = faceNormal[2];

        transformed_face_normals[i*3 + 0] = faceNormal[0];
        transformed_face_normals[i*3 + 1] = faceNormal[1];
        transformed_face_normals[i*3 + 2] = faceNormal[2];
    }

    normals_precalculated = true;
}

void Graphics3D::drawSolid(){
	const int cx = matrixPanel->getWidth()/2;
    const int cy = matrixPanel->getHeight()/2;
    
    int buf[face_count] = {0};
    float Zarray[face_count] = {0};
    
    // calculate z-buffer on every frame
    Zarr(transformed_vertices,faces,face_count,Zarray); //gets array of all the z positions of the faces and dump them in "Zarray"
	sort(Zarray,face_count,buf);        //get sorted indices of the z array to apply the "painter’s algorithm"

    // used to store the 2d projected 3d coordinates
    float px[VERTS_PER_FACE] = {0};
    float py[VERTS_PER_FACE] = {0};

    // shininess of the specular highlights (if enabled)
    float shininess = 10.0f;

    // color of the specular highlights (if enabled)
    float specularColor[3] = {1.0f, 1.0f, 1.0f}; //white

    float camNormal[3] = {cameraPos[0], cameraPos[1], cameraPos[2]};
    normalize(camNormal);

    float sunNormal[3] = {lightPos[0], lightPos[1], lightPos[2]};
    if (enable_diffuse || enable_specular) {
        normalize(sunNormal);
    }

    // iterate over every face
    for (int j=0;j<face_count;j++){
		
		int i = buf[j];

        // here is where the 3d coordinates are mapped onto a 2d xz surface
        for (int k=0; k<VERTS_PER_FACE; k++) {
            float x = transformed_vertices[faces[i*VERTS_PER_FACE+k]*3+0]+cameraPos[0];
            float y = transformed_vertices[faces[i*VERTS_PER_FACE+k]*3+1]+cameraPos[1];
            float z = transformed_vertices[faces[i*VERTS_PER_FACE+k]*3+2]+cameraPos[2];
            x *= zoom/y;
            z *= zoom/y;
            px[k] = cx+x;
            py[k] = cy+z;
        }

        // the following code are for the shaders
        //http://www.opengl-tutorial.org/beginners-tutorials/tutorial-8-basic-shading/

        // diffuse color of the 3d face
        float diffuseColor[3] = {face_colors[i*3 + 0], face_colors[i*3 + 1], face_colors[i*3 + 2]};

        // color of the ambient licht
        float ambientColor[3] = {face_colors[i*3 + 0]/10, face_colors[i*3 + 1]/10, face_colors[i*3 + 2]/10};
        
        // if normals are precalculated, get the normals, otherwise calculate them
        float faceNormal[3] = {0};
        if (normals_precalculated) {
            faceNormal[0] = transformed_face_normals[i*3 + 0];
            faceNormal[1] = transformed_face_normals[i*3 + 1];
            faceNormal[2] = transformed_face_normals[i*3 + 2];
        } else {
            getFaceNormal(vertices, faces, i, faceNormal);
        }        

        //some crude optimization: if the face faces away from the camera, don't draw it.
        if (dot(camNormal, faceNormal) > 0.0) {

            // common to diffuse and specular:
            // calculate the normal pointing to the light source and calculate the distance to the face
            float dist_sq = 1;
            if (enable_diffuse || enable_specular) {
                float centerFace[3] = {0};
                face_center(transformed_vertices, faces, i, centerFace);
                dist_sq = dist_squared(lightPos, centerFace);
            }

            // diffuse
            float cosTheta = 0;
            if (enable_diffuse) {
                //cosTheta = sin(clamp(dot(sunNormal,faceNormal),1,0));
                cosTheta = clamp(dot(sunNormal,faceNormal),1,0);
            }

            // specular
            float cosAlpha = 0;
            if (enable_specular) {
                float reflection[3] = {0};
                float lightRay[3] = {sunNormal[0], sunNormal[1], sunNormal[2]};
                reflect(lightRay, faceNormal, reflection);
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

            matrixPanel->fillQuat(px, py, color[0]*255, color[1]*255, color[2]*255, 1);
        }
    }
}

void Graphics3D::setVertices(float *verts, int size) {
    vertices = (float *) realloc(vertices, sizeof(float *)*size);
    transformed_vertices = (float *) realloc(transformed_vertices, sizeof(float *)*size);

    for (int i=0; i<size; i++) {
        vertices[i] = verts[i];
        transformed_vertices[i] = verts[i];
    }

    vert_count = size/3.0f;
}

void Graphics3D::setFaces(int *quats, int size) {
    faces = (int *) realloc(faces, sizeof(int *)*size);

    for (int i=0; i<size; i++) {
        faces[i] = quats[i];
    }

    face_count = size/4;
}

void Graphics3D::setFaceColors(uint8_t *colors, int size) {
    face_colors = (float *) realloc(face_colors, sizeof(float *)*size);

    for (int i=0; i<size; i++) {
        face_colors[i] = clamp(colors[i]/255.0f,1,0);
    }
}

void Graphics3D::setRotation(float x, float y, float z) {
    
    float cosx = cos(x);
    float sinx = sin(x);

    float cosy = cos(y);
    float siny = sin(y);

    float cosz = cos(z);
    float sinz = sin(z);

    float Axx = cosz*cosy;
    float Axy = cosz*siny*sinx - sinz*cosx;
    float Axz = cosz*siny*cosx + sinz*sinx;

    float Ayx = sinz*cosy;
    float Ayy = sinz*siny*sinx + cosz*cosx;
    float Ayz = sinz*siny*cosx - cosz*sinx;

    float Azx = -siny;
    float Azy = cosy*sinx;
    float Azz = cosy*cosx;

    float matrix[9] = {
        Axx, Axy, Axz, 
        Ayx, Ayy, Ayz, 
        Azx, Azy, Azz
    };

    // rotate vertices
    apply_matrix(vertices, transformed_vertices, matrix, vert_count);

    // if the normals are already calculated, we rotate then together with the vertices
    if (normals_precalculated) {
        apply_matrix(face_normals, transformed_face_normals, matrix, face_count);
    }
}

// draws the wireframe of the 3d model (all lines are drawn twice for now)
void Graphics3D::drawMesh(uint8_t r, uint8_t g, uint8_t b) {
    const int cx = matrixPanel->getWidth()/2;
    const int cy = matrixPanel->getHeight()/2;
    
    // used to store the 2d projected 3d coordinates
    float px[VERTS_PER_FACE] = {0};
    float py[VERTS_PER_FACE] = {0};

    // iterate over every face all
    for (int i=0;i<face_count;i++){

        // here is where the 3d coordinates are mapped onto a 2d xz surface
        for (int k=0; k<VERTS_PER_FACE; k++) {
            float x = transformed_vertices[faces[i*VERTS_PER_FACE+k]*3+0]+cameraPos[0];
            float y = transformed_vertices[faces[i*VERTS_PER_FACE+k]*3+1]+cameraPos[1];
            float z = transformed_vertices[faces[i*VERTS_PER_FACE+k]*3+2]+cameraPos[2];
            x *= zoom/y;
            z *= zoom/y;
            px[k] = cx+x;
            py[k] = cy+z;
        }

        matrixPanel->drawLineWu(px[0], py[0], px[1], py[1], r, g, b);
        matrixPanel->drawLineWu(px[1], py[1], px[2], py[2], r, g, b);
        matrixPanel->drawLineWu(px[2], py[2], px[3], py[3], r, g, b);
        matrixPanel->drawLineWu(px[3], py[3], px[0], py[0], r, g, b);
    }
}