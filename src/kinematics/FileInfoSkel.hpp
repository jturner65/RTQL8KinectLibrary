/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumitj83@gmail.com>, Sehoon Ha <sehoon.ha@gmail.com>
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

#ifndef KINEMATICS_FILEINFO_SKEL_H
#define KINEMATICS_FILEINFO_SKEL_H

#include <fstream>
#include <iomanip>
#include <string>
#include <iostream>
#include <sstream>

#include <Eigen/Dense>

#include "renderer/RenderInterface.h"
#include "Skeleton.h"
#include "BodyNode.h"
#include "ParserVsk.h"
#include "Marker.h"
#include "Dof.h"
#include "Joint.h"
#include "Marker.h"
#include "Transformation.h"
#include "ShapeEllipsoid.h"
#include "ShapeCube.h"

int readSkelFile( const char* const filename, rtql8::kinematics::Skeleton* skel );

namespace rtql8 {
    namespace kinematics {

        class BodyNode;
        class Skeleton;

        enum SkeletonFileType {
            VSK,
            SKEL
        };

        template <class SkeletonType>
        class FileInfoSkel {
        public:
            FileInfoSkel();
            virtual ~FileInfoSkel();

            void draw(renderer::RenderInterface* _ri = NULL, const Eigen::Vector4d& _color=Eigen::Vector4d::Ones(), bool _useDefaultColor = true) const; ///< Note: _color is not used when _useDefaultColor=false
            bool loadFile(const char* _filename);
            bool loadFile(const char* _filename, SkeletonFileType _type);
            bool saveFile(const char* _filename) const;

            SkeletonType* getSkel() const { return mSkel; }

        protected:
            SkeletonType* mSkel;
            char mFileName[256]; // could use string instead

            // used in saveFile to write the subtree of _b
            void saveBodyNodeTree(BodyNode* _b, std::ofstream &_outfile, int _numLinks) const;
        };

        template <class SkeletonType>
        FileInfoSkel<SkeletonType>::FileInfoSkel() : mSkel(NULL){
        }

        template <class SkeletonType>
        FileInfoSkel<SkeletonType>::~FileInfoSkel(){
            if(mSkel){
                delete mSkel;
                mSkel = NULL;
            }
        }

        template <class SkeletonType>
        void FileInfoSkel<SkeletonType>::draw(renderer::RenderInterface* _ri, const Eigen::Vector4d& _color, bool _useDefaultColor) const {
            mSkel->draw(_ri, _color, _useDefaultColor);
        }

        template <class SkeletonType>
        bool FileInfoSkel<SkeletonType>::loadFile( const char* _fName ) {

            std::string ext( _fName );
            ext = ext.substr(ext.length() - 4);

            if (ext == ".vsk") {
                return loadFile(_fName, VSK);
            } else if (ext == "skel") {
                return loadFile(_fName, SKEL);
            } else {
                std::cout << "invalid skeleton ext: " << ext;
                return false;
            }
        }


        template <class SkeletonType>
        bool FileInfoSkel<SkeletonType>::loadFile( const char* _fName, SkeletonFileType _type ) {
            if( mSkel )
                delete mSkel;
            // mSkel = new Skeleton;
            mSkel = new SkeletonType;

            bool success = false;
            if(_type == SKEL)
                success = !readSkelFile( _fName, mSkel);
            else if(_type == VSK)
                success = (readVSKFile(_fName, mSkel) == VSK_OK);
            if(success){
                std::string text = _fName;
                int lastSlash = text.find_last_of("/");
                text = text.substr(lastSlash+1);
                strcpy(mFileName, text.c_str());
            }
            return success;
        }

        const static std::string unitlength = "unitlength";

        template <class SkeletonType>
        bool FileInfoSkel<SkeletonType>::saveFile( const char* _fName) const {
            using namespace std;
            ofstream output(_fName);
            if(output.fail()){
                std::cout << "Unable to save file "<< _fName << std::endl;
                return false;
            }
            output<<fixed<<setprecision(6);

            int _numLinks = mSkel->mNumNodes;

            output<<"dofs {\n";
            for(int i=0; i<mSkel->getNumDofs(); i++){
                Dof *d = mSkel->getDof(i);
                if(d->getJoint()->getChildNode()->getSkelIndex() >= _numLinks) break;
                output<<'\t'<<d->getName()<<" { "<<d->getValue()<<", "<<d->getMin()<<", "<<d->getMax()<<" }\n";
            }
            output<<unitlength<<" { 1.0, 1.0, 1.0 }\n";
            output<<"}\n";

            // masses
            output<<"\nmass {\n";
            for(int i=0; i<mSkel->mNumNodes; i++){
                if(i>=_numLinks) break;
                string massname = mSkel->getNode(i)->getName();
                massname += "_mass";
                output<<massname<<" { "<<mSkel->getNode(i)->getMass()<<" }\n";

            }
            output<<"}\n";

            // nodes
            saveBodyNodeTree(mSkel->mRoot, output, _numLinks);

            // markers
            output<<"\nmarkers {\n";
            for(int i=0; i<mSkel->getNumMarkers(); i++){
                if(mSkel->getMarker(i)->getNode()->getSkelIndex()>=_numLinks) break;
                string hname = mSkel->getMarker(i)->getName();
                if(hname.empty()){
                    ostringstream ss;
                    ss<<"marker"<<i;
                    hname = ss.str();
                }
                Eigen::Vector3d lc = mSkel->getMarker(i)->getLocalCoords();
                output<<hname<<" { "<<"<"<<lc[0]<<", "<<lc[1]<<", "<<lc[2]<<">, "<<i<<", "<<mSkel->getMarker(i)->getNode()->getName()<<" } "<<endl;

            }
            output<<"}\n";

            std::cout <<"\n";

            return true;
        }

        template <class SkeletonType>
        void FileInfoSkel<SkeletonType>::saveBodyNodeTree(BodyNode *_b, std::ofstream &_outfile, int _numLinks) const {
            // save the current one
            _outfile<<"\nnode "<<_b->getName()<<" { "<<_b->getSkelIndex()<<"\n";
            // write the trans
            _outfile<<"chain { "<<_b->getParentJoint()->getNumTransforms()<<"\n";
            for(int i=0; i<_b->getParentJoint()->getNumTransforms(); i++){
                Transformation *tr = _b->getParentJoint()->getTransform(i);
                if(!tr->getVariable()){	// constant
                    if(tr->getType()==Transformation::T_TRANSLATE){
                        _outfile<<"telescope { <"<<tr->getDof(0)->getValue()<<", "<<tr->getDof(1)->getValue()<<", "<<tr->getDof(2)->getValue()<<">, "<<unitlength<<" }\n";
                    }
                    else if (tr->getType()==Transformation::T_ROTATEX){
                        _outfile<<"rotate_cons { "<<tr->getDof(0)->getValue()<<", "<<" x }\n";
                    }
                    else if (tr->getType()==Transformation::T_ROTATEY){
                        _outfile<<"rotate_cons { "<<tr->getDof(0)->getValue()<<", "<<" y }\n";
                    }
                    else if (tr->getType()==Transformation::T_ROTATEZ){
                        _outfile<<"rotate_cons { "<<tr->getDof(0)->getValue()<<", "<<" z }\n";
                    }
                    else {
                        _outfile<<"unknown trans\n";
                    }

                }
                else {	// variable
                    if(tr->getType()==Transformation::T_TRANSLATE){
                        _outfile<<"translate { <"<<tr->getDof(0)->getName()<<", "<<tr->getDof(1)->getName()<<", "<<tr->getDof(2)->getName()<<"> }\n";
                    }
                    else if (tr->getType()==Transformation::T_ROTATEX){
                        _outfile<<"rotate_euler { "<<tr->getDof(0)->getName()<<", "<<" x }\n";
                    }
                    else if (tr->getType()==Transformation::T_ROTATEY){
                        _outfile<<"rotate_euler { "<<tr->getDof(0)->getName()<<", "<<" y }\n";
                    }
                    else if (tr->getType()==Transformation::T_ROTATEZ){
                        _outfile<<"rotate_euler { "<<tr->getDof(0)->getName()<<", "<<" z }\n";
                    }
                    else if (tr->getType()==Transformation::T_ROTATEEXPMAP){
                        _outfile<<"rotate_expmap { <"<<tr->getDof(0)->getName()<<", "<<tr->getDof(1)->getName()<<", "<<tr->getDof(2)->getName()<<"> }\n";
                    }
                    else {
                        _outfile<<"unknown trans\n";
                    }
                }
            }
            _outfile<<"}\n";	// chain

            // primitive
            Eigen::Vector3d pdim = _b->getShape()->getDim();
            Eigen::Vector3d off = _b->getLocalCOM();
            _outfile<<"primitive { <"<<pdim[0]<<", "<<pdim[1]<<", "<<pdim[2]<<">, <"<<off[0]<<", "<<off[1]<<", "<<off[2]<<">, "<<unitlength;
            // different types

            Shape* prim = _b->getShape();
            ShapeEllipsoid* elp = dynamic_cast<ShapeEllipsoid*>(prim);
            ShapeCube* box = dynamic_cast<ShapeCube*>(prim);

            if(elp) _outfile<<", SPHERE";
            else if(box) _outfile<<", CUBE";

            _outfile<<", "<<std::string(_b->getName())+std::string("_mass");
            _outfile<<" }\n";

            for(int i=0; i<_b->getNumChildJoints(); i++){
                if(_b->getChildNode(i)->getSkelIndex()>=_numLinks) continue;
                saveBodyNodeTree(_b->getChildNode(i), _outfile, _numLinks);
            }

            _outfile<<"}\n";	//node

        }


    } // namespace kinematics
} // namespace rtql8

#endif // #ifndef KINEMATICS_FILEINFO_SKEL_H

