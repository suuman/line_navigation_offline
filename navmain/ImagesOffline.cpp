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


#include "ImagesOffline.h"

#include <iostream>
#include <chrono>
#include <thread>

    ImagesOffline::ImagesOffline(const std::string &opt_ip):RobotInterface(opt_ip){

         K = (cv::Mat_<double>(3, 3) <<296.3437428191,0,158.2733787287, 0, 296.6441607237,118.0622191474 , 0, 0, 1);

         // Start pepper control
         odom.push_back(0.0);
         odom.push_back(0.0);
         odom.push_back(0.0);

         imgfold = opt_ip;
        // imgfold += "/";
         indx = 0;

         destid = 0;

    }

    void ImagesOffline::openCamera(int id){

    cv::glob(imgfold + "*.png",imlist);
    destid = id;

    }



    cv::Mat ImagesOffline::getK(){
        return K.clone();
    }


    void ImagesOffline::startRobot(int id = 0){

        openCamera(id);


    }



    void ImagesOffline::getCurrImage(cv::Mat &I)
    {
        I = cv::imread(imlist.at(indx));
        indx++;
    }

    void  ImagesOffline::setBaseVelocities(float vr, float wr){


    }



    std::vector<float> ImagesOffline::getOdometryReading(){

       return (odom);
    }

    void ImagesOffline::rotate180()
    {
    }

    void ImagesOffline::wait(long t){

        std::this_thread::sleep_for(std::chrono::milliseconds(t));
    }

    int ImagesOffline::getmode(){
        return 1;
    }

    int ImagesOffline::getid(){
        return(destid);
    }
