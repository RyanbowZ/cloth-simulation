#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

shared_ptr<Spring> createSpring(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, double E)
{
	auto s = make_shared<Spring>(p0, p1);
	s->E = E;
	Vector3d x0 = p0->x;
	Vector3d x1 = p1->x;
	Vector3d dx = x1 - x0;
	s->L = dx.norm();
	return s;
}

Cloth::Cloth(int rows, int cols,
			 const Vector3d &x00,
			 const Vector3d &x01,
			 const Vector3d &x10,
			 const Vector3d &x11,
			 double mass,
			 double stiffness)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);
	
	this->rows = rows;
	this->cols = cols;
    
	//
	// Create particles here
	//
	this->n = 0; // size of global vector (do not count fixed vertices)
	double r = 0.01; // Used for collisions
	int nVerts = rows*cols;
    
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = r;
            p->m = mass/nVerts;
			
            Vector3d xij;
            double s=j/(cols-1.0f),t=i/(rows-1.0f);
            xij=(1-s)*(1-t)*x00+s*(1-t)*x01+s*t*x11+(1-s)*t*x10;
            p->x=xij;
            
			// Populate the other member variables of p here
            if((!i&&!j)||(!i&&j==cols-1)){
                p->i=-1;
                p->fixed=true;
            }
            else{
                p->i=this->n;
                this->n+=3; //
                p->fixed=false;
            }
		}
	}
	
	//
	// Create springs here
	//

    vector<pair<int,int>> dir{{1,0},{0,1},{2,0},{0,2},{1,1},{-1,1}};
    for(int i = 0; i < rows; ++i) {
        for(int j = 0; j < cols; ++j){
            for(auto &[m,n]:dir){
                int in=i+n,jm=j+m;
                if(in>=0&&in<rows&&jm>=0&&jm<cols){
                    auto s=createSpring(particles[i*cols+j], particles[in*cols+jm], stiffness);
                    springs.push_back(s);
                }
            }
        }
    }

	// Allocate system matrices and vectors
	M.resize(n,n);
	K.resize(n,n);
	v.resize(n);
	f.resize(n);
    
    kv=vector<vector<double>>(n,vector<double>(n,0));
    kz=vector<double>(n,0);
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();

	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}

	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Cloth::reset()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePosNor();
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x;
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = x(2);
		}
	}
	
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     /|\
			// u0 /_|_\ u1
			//    \ | /
			//     \|/
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = nor(2);
		}
	}
}

void Cloth::step(double h, const Vector3d &grav, const vector< shared_ptr<Particle> > spheres)
{
    
    vector<T> M_, K_;
    
	v.setZero();
	f.setZero();

    fill(kv.begin(),kv.end(),kz);

    Matrix3d i3;
    i3.setIdentity();

    for(auto p: particles){
        int si=p->i;
        if(si!=-1){
            M_.emplace_back(T(si,si,p->m));
            M_.emplace_back(T(si+1,si+1,p->m));
            M_.emplace_back(T(si+2,si+2,p->m));
            v.segment<3>(si)=p->v;
            f.segment<3>(si)=p->m*grav;
        }
    }
    
    for(auto s:springs){
        auto p0=s->p0, p1=s->p1;
        Vector3d dx=p1->x-p0->x;
        double l=dx.norm(), L=s->L, E=s->E;
        Vector3d fs=E*(l-L)/l*dx;
        int p0i=p0->i, p1i=p1->i;
        if(p0i!=-1)f.segment<3>(p0i)+=fs;
        if(p1i!=-1)f.segment<3>(p1i)-=fs;
        Matrix3d Ks=E/pow(l,2)*(L/l*dx*dx.transpose()+(1-L/l)*dx.transpose()*dx*i3);

        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                double Ksij=Ks(i,j);
                if(Ksij==0)continue;
                if(p0i!=-1)kv[p0i+i][p0i+j]-=Ksij;
                if(p1i!=-1)kv[p1i+i][p1i+j]-=Ksij;
                if(p0i!=-1&&p1i!=-1){
                    kv[p0i+i][p1i+j]+=Ksij;
                    kv[p1i+i][p0i+j]+=Ksij;
                }
            }
        }
        
    }
    
//     Sphere Collision
    double c=8;
    auto sp = spheres[0];
    for(auto p:particles){
        
        Vector3d dx = p->x-sp->x;
        double l=dx.norm();
        
        double d=p->r+sp->r-l;
        if(d>0){
            int si=p->i;
            double cd=c*d;
            if(si!=-1){
                Vector3d n=dx/l;
                f.segment<3>(si)+=cd*n;

                kv[si][si]+=cd;
                kv[si+1][si+1]+=cd;
                kv[si+2][si+2]+=cd;
            }
        }
    }
    
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            double kvij=kv[i][j];
            if(kvij!=0)K_.emplace_back(T(i,j,kvij));
        }
    }

    M.setFromTriplets(M_.begin(), M_.end());
    K.setFromTriplets(K_.begin(), K_.end());
    SpMat A=M-pow(h,2)*K;
    VectorXd b=M*v+h*f;
    ConjugateGradient< SparseMatrix<double> > cg;
    cg.setMaxIterations(25);
    cg.setTolerance(1e-6);
    cg.compute(A);
    VectorXd x = cg.solveWithGuess(b, v);
    
    for (auto p:particles){
        int si=p->i;
        if(si!=-1){
            Vector3d v2=x.segment<3>(si);
            p->v=v2;
            p->x+=h*v2;
        }
    }
    
    
	// Update position and normal buffers
	updatePosNor();

}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3fv(p->getUniform("kdFront"), 1, Vector3f(1.0, 0.0, 0.0).data());
	glUniform3fv(p->getUniform("kdBack"),  1, Vector3f(1.0, 1.0, 0.0).data());
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	int h_pos = p->getAttribute("aPos");
	glEnableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("aNor");
	glEnableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	glDisableVertexAttribArray(h_nor);
	glDisableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}
