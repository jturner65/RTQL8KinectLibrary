/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Georgia Tech Graphics Lab
 * 
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * This code incorporates portions of Open Dynamics Engine 
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights 
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow 
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GEOMETRY_MESH3D_H
#define GEOMETRY_MESH3D_H

#include <vector>

using namespace std;
#include <Eigen/Dense>

namespace rtql8 {
    namespace geometry {
        class Mesh3D{
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            enum MeshFormat {OBJ, OFF, CORNERTABLE};
            char mFileName[1024];

            unsigned int mNumVertices;
            unsigned int mNumFaces;
            vector<unsigned int> mNumFaceVertices;

            vector<Eigen::Vector2d> mVertexTextures;
            vector<Eigen::Vector3d> mVertexNormals;	// may not be same as the number of vertices

            Eigen::VectorXd mVertexPos;	// xyz in sequence
            Eigen::VectorXd mVertexVel;	// xyz in sequence
            vector<int> mFaces;	// list of 3 or more vertex indices in sequence for each face
            vector<vector<int> > mFaceTextureIndices;
            vector<vector<int> > mFaceNormalIndices;	// should exactly correspond to mFaces; indexes into the mVertexNormals

            Mesh3D(){
                mNumVertices = 0;
                mNumFaces=0;
                mVertexPos = Eigen::VectorXd();
                mVertexVel = Eigen::VectorXd();
                mVertexTextures.clear();
                mFaces.clear();
                mFileName[0]='\0';
            }
            virtual bool readMesh(const char *_file, MeshFormat _format);
            virtual bool writeMesh(const char *_file, MeshFormat _format);

            virtual double computeVolume()=0;

            virtual void draw(const Eigen::Vector4d& _color, bool _drawWireFrame, bool _drawSmooth=true)=0;

        };  // Mesh3D

    } // namespace geometry
} // namespace rtql8

#endif  //GEOMETRY_MESH3DGEN_H

