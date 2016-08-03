/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): John Turner <jturner65@gatech.edu>
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

#include "KinHndlrConsts.h"
#include "KinUIImageButton.h"

using namespace std;

namespace rtql8{
    namespace kinect{

        int KinUIImageButton::reInit(){			//any external-specific functionality
            KinUIComponent::reInit();		    //parent reinit/init
            init();
            return 0;
        }

        void KinUIImageButton::init(){
            if((w > 0) && (h  > 0)){
                const BYTE val = 0x01;																	//initialize copy of valid data ara for image
                std::memset(imgData, val, w*h*4);
            }		
            flags[UIobjIDX_OffsLbl] = true;                                                             //any text label associated with this should be offset, so as to not cover the image
        }//init

        //draw this component on the screen
        void KinUIImageButton::draw(){
            if(useAnimLoop) {           //use animation loop for button display
                if(flags[UIobjIDX_DispHS]){drawHotSpot();}
                drawAnimButton();
                // drawText(0, -.7*h, label);
                drawText(0, -.7*h, label);
            }
            else if (useFileSystem) {  //single image from file source     
                if(!imgLoaded){//load image - should be certain to have opengl context by here
                    this->instImgBtn();
                }
                if(!imgLoaded){         //check after load to be certain load succeeded
                    clog<<"KinUIImageButton : SOIL image load failure for obj ID : "<<this->ID<<endl;
                    return;
                } else {                //image loaded, display
                    if(flags[UIobjIDX_DispHS]){drawHotSpot();}
                    drawImageButton();
                    // drawText(0, -.7*h, label);
                    drawText(0, 0.0 * h, label);
                }
            }
            else {                  //single image from KIN - TODO
            }
        }//draw

        void KinUIImageButton::setImgSrcFileName(string _src){
            if("" == _src){return;}
            imgSrc = _src;                                                  //directory source for image
            imgLoaded = false;
            useFileSystem = true;                                           //file name given, using filesystem
        }//setImgSrc      

        //instantiate image button via SOIL library
 	       // Loads an image from disk into an OpenGL texture.
	        //\param filename the name of the file to upload as a texture
	        //\param force_channels 0-image format, 1-luminous, 2-luminous/alpha, 3-RGB, 4-RGBA                                           SOIL_LOAD_RGBA
	        //\param reuse_texture_ID 0-generate a new texture ID, otherwise reuse the texture ID (overwriting the old texture)           reuse
	        //\param flags can be any of SOIL_FLAG_POWER_OF_TWO | SOIL_FLAG_MIPMAPS | SOIL_FLAG_TEXTURE_REPEATS | SOIL_FLAG_MULTIPLY_ALPHA | SOIL_FLAG_INVERT_Y | SOIL_FLAG_COMPRESS_TO_DXT | SOIL_FLAG_DDS_LOAD_DIRECT
	        //\return 0-failed, otherwise returns the OpenGL texture handle
       void KinUIImageButton::instImgBtn(){
            try{
                int loadFlags = SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT;
                if(0 < imgTextureID ){
                    loadFlags = SOIL_FLAG_DDS_LOAD_DIRECT;
                } 
                //imgTextureID = SOIL_load_OGL_texture( const_cast<const char*>(imgSrc.c_str()), SOIL_LOAD_AUTO, imgTextureID, loadFlags);
                imgTextureID = SOIL_load_OGL_texture( imgSrc.c_str(), SOIL_LOAD_AUTO, imgTextureID, loadFlags);
                imgClickTextureID = SOIL_load_OGL_texture( const_cast<const char*>(imgSrc.c_str()), SOIL_LOAD_L, imgClickTextureID, loadFlags);
                if(( 0 == imgTextureID ) || ( 0 == imgClickTextureID)){
	               clog<<"KinUIImageButton : SOIL loading error: "<< SOIL_last_result()<<" file name : "<<imgSrc<<endl;
                }
                imgLoaded = true;
            } catch (const std::exception& ex){
                clog<<"KinUIImageButton : error loading image button texture : name:`"<<imgSrc<<"`"<<endl; 
                imgLoaded = false;
            }
        }//instImgBtn

        //draw image loaded from filesystem
        void KinUIImageButton::drawImageButton(){
            GLuint dispTexID;
            glPushMatrix();
                glTranslatef(x,y,0);
                glColor4f(1,1,1,1);	
                if(flags[UIobjIDX_MSDown]){ dispTexID = imgClickTextureID;	} 
                else {			            dispTexID = imgTextureID;	}				
                
                glEnable(GL_TEXTURE_2D);
                glDisable(GL_LIGHTING);
                glBindTexture(GL_TEXTURE_2D, dispTexID);
                glBegin( GL_QUADS);
                    glTexCoord2f(0, 1);glVertex2f(0, 0);				
                    glTexCoord2f(1, 1);glVertex2f(w, 0);				
                    glTexCoord2f(1, 0);glVertex2f(w, h);		        
                    glTexCoord2f(0, 0);glVertex2f(0, h);			    
                glEnd();
                glDisable(GL_TEXTURE_2D);
                glEnable(GL_LIGHTING);
            glPopMatrix();
        }//drawImageButton      

        //void KinUIImageButton::drawImageButton(){
        //    int retVal = 0;
        //    glPushMatrix();
        //    glTranslatef(x,y,0);

        //    if(flags[UIobjIDX_MSDown]){	glColor4f(1.0f-clr[0], 1.0f - clr[1], 1.0f - clr[2],clr[3]);} 
        //    else {			glColor4f(clr[0],clr[1],clr[2],clr[3]);	}				
        //        
        //    glScalef(.01f,.01f,1.0f);	
        //    glEnable(GL_TEXTURE_2D);
        //    glDisable(GL_LIGHTING);
        //    glBindTexture(GL_TEXTURE_2D, imgTextureID);
        //    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);    
        //    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);    
        //    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 1, GL_RGBA, GL_UNSIGNED_BYTE, (GLvoid*)imgData);	

        //    glBegin( GL_QUADS);
        //    glVertex3f(0, 0, 0);				glTexCoord2f(0.0f, 0.0f);
        //    glVertex3f(w, 0, 0);				glTexCoord2f(0.0f, -1.0f);
        //    glVertex3f(w, h, 0.0f);		        glTexCoord2f(-1.0f, -1.0f);
        //    glVertex3f(0, h, 0.0f);			    glTexCoord2f(-1.0f, 0.0f);
        //    glEnd();
        //    glDisable(GL_TEXTURE_2D);
        //    glEnable(GL_LIGHTING);
        //    glPopMatrix();
        //}//drawImageButton      

        void KinUIImageButton::drawAnimButton(){
            int retVal = 0;
            glPushMatrix();
                glTranslatef(x,y,0);

                if(flags[UIobjIDX_MSDown]){	glColor4f(1.0f-clr[0], 1.0f - clr[1], 1.0f - clr[2],clr[3]);} 
                else {			glColor4f(clr[0],clr[1],clr[2],clr[3]);	}				
                
                glScalef(.01f,.01f,1.0f);	
                glEnable(GL_TEXTURE_2D);
                glDisable(GL_LIGHTING);
                glBindTexture(GL_TEXTURE_2D, loopTexIDs[drawIdx]);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);    
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);    
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 1, GL_RGBA, GL_UNSIGNED_BYTE, (GLvoid*)(animLoop[drawIdx]));	

                glBegin( GL_QUADS);
                    glVertex3f(0, 0, 0);				glTexCoord2f(0.0f, 0.0f);
                    glVertex3f(w, 0, 0);				glTexCoord2f(0.0f, -1.0f);
                    glVertex3f(w, h, 0.0f);		        glTexCoord2f(-1.0f, -1.0f);
                    glVertex3f(0, h, 0.0f);			    glTexCoord2f(-1.0f, 0.0f);
                glEnd();
                glDisable(GL_TEXTURE_2D);
                glEnable(GL_LIGHTING);

                drawIdx = (drawIdx + 1)%numFrames;
            glPopMatrix();
        }//drawAnimButton

        //handles kinect event
        void KinUIImageButton::getValueForEvent(int hand, int type, int drag, float& val){       }	

        //handles kinect event
        int KinUIImageButton::click(float _x, float _y, int cmpObj){	return ID;}

        std::ostream& operator<<(std::ostream& out, const KinUIImageButton& cmp){
            out<<static_cast<const KinUIComponent&>(cmp);                                                         //start with base class

            return out;
        }//op<<

    }//namespace kinect
}//namespace rtql8
