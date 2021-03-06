/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
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

#ifndef UTILS_TIMER_H
#define UTILS_TIMER_H

#if WIN32
#include <windows.h>
typedef struct {
    LARGE_INTEGER  start;
    LARGE_INTEGER  stop;
} stopWatch;
#endif

namespace rtql8 {
    namespace utils {
        class Timer;

        /**
           @brief The implementation of Timer class

           This is a definition of mTimer class.
           For measure the time, clock() api is used
        */
        class Timer {
        public:
            Timer(const char* name); ///< Default constructor. The name can be up to 64
            ~Timer(); ///< Default destructor

            void startTimer();
            double elapsed(); ///< return elapsed time in seconds since startTimer().
            ///< @see startTimer()
            double lastElapsed() const;
            void stopTimer();
            bool isRunning() const;
            void printLog();
            void printScreen();
            void print( bool _toScreen=true );
            int getCount() { return mCount; }
            double getAve() { return mTotal / mCount; }

        private:
            int mCount;
    #if WIN32
            stopWatch mTimer;
    #else
            double mStart;
            double mStop;
    #endif
            double mLastElapsed;
            double mTotal;
            char *mName;
            bool mIsRunning;

    #if WIN32
            LARGE_INTEGER  mFrequency;
            double LIToSecs( LARGE_INTEGER & L) ;
    #endif
        };
    } // namespace utils
} // namespace rtql8

#endif // #ifndef UTILS_TIMER_H
