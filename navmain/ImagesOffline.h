/* Interface with for offline mode
 * higher level functionlities
 * Read images from the folder and do image based loclization only
 * Pepper robot not use
 * for image based loclaiztion only
 * for debugging purposes
 *
 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
/*
 * Author: Suman Raj Bista
 */

#ifndef IMAGESOFFLINE_H
#define IMAGESOFFLINE_H

#include "RobotInterface.h"

class ImagesOffline: public RobotInterface
{
private:
    cv::Mat I;
    cv::Mat K;
 
    std::vector<float> odom;
    std::string imgfold;
    std::vector<cv::String> imlist;
    
    int indx;
    int destid;
public:

    /**
         * @brief ImagesOffline
         * @param opt_ip = path of the folder where navigation image lies
         */
        ImagesOffline(const std::string &opt_ip);

        /**
         * @brief openCamera reads image file names from the folder
         * @param id
         */


        void openCamera(int id);

        /**
         * @brief startRobot  initialies image path for image-based localizations
         * @param id
         */
        void startRobot(int id);

        /**
         * @brief getCurrImage reads image from folder
         * @param I
         */
        void getCurrImage(cv::Mat &I);

        /* they are just empty for compatibility */

        void setBaseVelocities(float vr, float wr);
        std::vector<float> getOdometryReading();
        void rotate180();

        /**
         * @brief wait delay in ms
         */
        void wait(long t);

        /**
         * return intrinsic parameters
         */

        cv::Mat getK();

        /**
         * @brief getmode tells it is offline mode
         * @return 1
         */
        int getmode();

        /* they are just empty for compatibility */
        int getid();

};

#endif // PEPPEROFFLINE_H
