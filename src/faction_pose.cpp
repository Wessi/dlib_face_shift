#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <math.h>
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <unistd.h>

using namespace std;
using namespace cv;
using namespace dlib;


double lmPoints[68][2];
double first100ds[500] = {};
double intermediateaR[500] = {};
double max_value = 0, cur = 0;
double verticesLE[68][2] = {{}};
double verticesRE[68][2] = {{}};
double lipsSmileL[68][2] = {{}};
double lipsSmileR[68][2] = {{}};
double verticesIO[68][2] = {{}};
double lipsSmileLi[68][2] = {{}};
double lipsSmileRi[68][2] = {{}};
double coordinates[68][2] = {{3,4}, {5,11}, {12,8}, {9,5},{5,6}};
bool faceDetected = false;
bool captured = false;
int cnt = 0; 
int icnt = 0;
int s1, s2, s3, s4, s5, s6;
double first100dsAvg = 0;
double bv = 0;
double roundOff_bv = 0.000;

dlib::image_window win;
dlib::frontal_face_detector detector;
dlib::shape_predictor pose_model;
ros::Publisher pub;

double areaOfShape(double points[68][2], long rows){
    double sum = 0, area = 0;
    for(long r=0; r<rows; r++){
        if(r == rows-1){
            sum += points[r][0]*points[0][1] - points[0][0]*points[r][1];
        }else {
            sum += points[r][0]*points[r+1][1] - points[r+1][0]*points[r][1];
        }
    }
    area = abs((long)sum)/2;
    return area;
}




void imageCB(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cvPtr;
    try {
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    dlib::cv_image<dlib::bgr_pixel> cimg(cvPtr->image);

    // Detect faces 
    std::vector<dlib::rectangle> faces = detector(cimg);
    // Find the pose of each face.
    std::vector<dlib::full_object_detection> shapes;
    for (unsigned long i = 0; i < faces.size(); ++i)
        shapes.push_back(pose_model(cimg, faces[i]));

    // Display it all on the screen
    win.clear_overlay();
    win.set_image(cimg);
    win.add_overlay(render_face_detections(shapes));

    

    if(shapes.size()>0){
                 for (unsigned long i = 0; i < shapes.size(); ++i)
                {                    
                    const full_object_detection& d = shapes[i];
                    for (unsigned long i = 0; i < 68; ++i)
                    {
                        lmPoints[i][0] = (double)d.part(i).x();
                        lmPoints[i][1] = (double)d.part(i).y();
                        //std::cout<<"i: "<<i<<" X: "<<lmPoints[i][0]<<"="<<d.part(i).x()<<" Y: "<<lmPoints[i][1]<<"="<<d.part(i).y()<<std::endl; 
                        // if (!faceDetected)
                        // {
                        //     faceDetected = true;            
                        // }
                    }
                    //std::cout<<std::endl; 
                }
                   /*
                    find the area of polygon
                    for each value that are obtained by different eye or mouse actions
                    publish some action on the screen 
                    eg if(le<re --> write left eye close... etc)
                    */
                    double diffLeftEye = lmPoints[47][1] - lmPoints[43][1];
                    double diffRightEye = lmPoints[40][1] - lmPoints[38][1];  
                    double s2 = lmPoints[38][0] - lmPoints[37][0];  
                    double s5 = lmPoints[41][0] - lmPoints[40][0];  

                    // cout<<"Side2: test my code @icog: "<<s2<<endl;
                    // cout<<"Side5: test my code @icog: "<<s5<<endl;
               
                    // cout<<"\nLeft eye: test my code @icog: "<<diffLeftEye<<endl;
                    // cout<<"\nRight eye: test my code @icog: "<<diffRightEye<<endl;

                    //vertices for right and left eye
                    long vsize = 0;
                    for (int vr = 36, vl = 42; vr<42, vl<48; vr++, vl++)
                    {
                        verticesRE[vsize][0] = lmPoints[vr][0];
                        verticesRE[vsize][1] = lmPoints[vr][1];

                        verticesLE[vsize][0] = lmPoints[vl][0];
                        verticesLE[vsize][1] = lmPoints[vl][1];

                        vsize++;
                    }
                    // https://www.youtube.com/watch?v=t9FzskM3P_0#t=58
                    // cout<<"\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n";
                    // cout<<"\nHello Outside-vsize "<<vsize<<" Right AREA:"<<areaOfShape(verticesRE, vsize);
                    // cout<<"\nHello Outside-vsize "<<vsize<<" Left AREA:"<<areaOfShape(verticesLE, vsize);
                    // cout<<"\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n";

                    //vertices for Inner Opening
                    long msize = 0;
                    for (int m = 60; m<68; m++)
                    {
                        verticesIO[msize][0] = lmPoints[m][0];
                        verticesIO[msize][1] = lmPoints[m][1];
                        msize++;
                    }

                    //vertices for lipsSmileROut
                    lipsSmileR[0][0] = lmPoints[59][0];
                    lipsSmileR[0][1] = lmPoints[59][1];

                    lipsSmileR[1][0] = lmPoints[48][0];
                    lipsSmileR[1][1] = lmPoints[48][1];

                    lipsSmileR[2][0] = lmPoints[49][0];
                    lipsSmileR[2][1] = lmPoints[49][1];

                    lipsSmileR[3][0] = lmPoints[50][0];
                    lipsSmileR[3][1] = lmPoints[50][1];
                    
                    lipsSmileR[4][0] = lmPoints[60][0];
                    lipsSmileR[4][1] = lmPoints[60][1];

                    lipsSmileR[5][0] = lmPoints[61][0];
                    lipsSmileR[5][1] = lmPoints[61][1];

                    lipsSmileR[6][0] = lmPoints[67][0];
                    lipsSmileR[6][1] = lmPoints[67][1];

                    //vertices for lipsSmileLOut
                    lipsSmileL[0][0] = lmPoints[55][0];
                    lipsSmileL[0][1] = lmPoints[55][1];

                    lipsSmileL[1][0] = lmPoints[54][0];
                    lipsSmileL[1][1] = lmPoints[54][1];

                    lipsSmileL[2][0] = lmPoints[53][0];
                    lipsSmileL[2][1] = lmPoints[53][1];

                    lipsSmileL[3][0] = lmPoints[52][0];
                    lipsSmileL[3][1] = lmPoints[52][1];

                    lipsSmileL[4][0] = lmPoints[64][0];
                    lipsSmileL[4][1] = lmPoints[64][1];

                    lipsSmileL[5][0] = lmPoints[63][0];
                    lipsSmileL[5][1] = lmPoints[63][1];

                    lipsSmileL[6][0] = lmPoints[65][0];
                    lipsSmileL[6][1] = lmPoints[65][1];
                    //  //vertices for lipsSmileRInner
                    // lipsSmileRi[0][0] = lmPoints[60][0];
                    // lipsSmileRi[0][1] = lmPoints[60][1];

                    // lipsSmileRi[1][0] = lmPoints[61][0];
                    // lipsSmileRi[1][1] = lmPoints[61][1];

                    // lipsSmileRi[2][0] = lmPoints[67][0];
                    // lipsSmileRi[2][1] = lmPoints[67][1];

                     //vertices for lipsSmileRInner
                    // lipsSmileLi[0][0] = lmPoints[64][0];
                    // lipsSmileLi[0][1] = lmPoints[64][1];

                    // lipsSmileLi[1][0] = lmPoints[63][0];
                    // lipsSmileLi[1][1] = lmPoints[63][1];

                    // lipsSmileLi[2][0] = lmPoints[65][0];
                    // lipsSmileLi[2][1] = lmPoints[65][1];


                    cout<<"\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n";
                    // cout<<"\nlipsSmileR Nose @31: ("<<lmPoints[31][0]<<", "<<lmPoints[31][1]<<")";
                    double aR = areaOfShape(lipsSmileR, 7);
                    double aL = areaOfShape(lipsSmileL, 7);
                    // if(aR>aL)
                    // {
                        cout<<"\nArea of lipsSmileR: "<<aR;
                    // }
                    // else if (aL>aR)
                    // {
                    //     cout<<"\nArea of lipsSmileL: "<<aL;
                    // }
                    cout<<"\n@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n";
                    // cout<<"\nArea of lipsSmileL: "<<aL;
                    // cout<<"\nArea of lipsSmileR: "<<aR;
                    //start capturing first 100ds               
                    if (cnt < 100)
                    {
                        first100ds[cnt++] = aR;
                        cout<<"\ncnt:"<<cnt<<"\n";
                    }
                    else if (cnt == 100)
                    {
                        captured = true;
                        cout<<"\ncaptured the first "<<cnt<<" values\n";
                        cnt++;
                    }  
                     //here start code for finding average of captured
                    if(captured){
                        double sum = 0;
                        for (int i = 0; i < 100; ++i)
                        {
                            sum += first100ds[i];
                        }
                        first100dsAvg = sum/100;
                        cout<<"\n miiiiiiiiiiiiiiiiiiiiiiiiiiinnnnnnnnnnnn:"<<first100dsAvg<<"\n";
                        captured = false;                        
                        // cnt = 0;
                    }  

                    cur = aR;
                    if (cur > first100dsAvg && cur > max_value && cnt >= 100)
                    {
                        max_value = cur;
                        //here i have the max_valueimum as well as the minimum so do the trick
                        icnt = 0;
                        cout<<"\n max_valuexxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx:"<<max_value<<"\n";

                    }else if (cur>first100dsAvg && cur < max_value && cnt >= 100)
                    {
                        //here push all the intermediate values to the array intermediateaR
                        //dont store but calculate
                        //normalize the cur value to 0-1 range i.e. if cur is in between (min, max_value) range
                        bv = (cur - first100dsAvg)/(max_value - first100dsAvg);
                        roundOff_bv = ceil( ( bv * pow( 10, 3) ) - 0.49 ) / pow( 10, 3);

                        intermediateaR[icnt] = cur;
                        cout<<"\nThe Blendeer value:"<<roundOff_bv<<endl;


                        // Publish
                        std_msgs::String msg;
                        std::stringstream ss;
                        ss << "The blender value is " << roundOff_bv<<endl;
                        msg.data = ss.str();
                        pub.publish(msg);

                        icnt++;
                    }                   

                //     faceDetected = false;
            } else{
                cnt = 0;
                icnt = 0;
                // cout<<"\ncnt:"<<cnt<<endl;
            }   

   
}

int main(int argc, char **argv) {

    detector = dlib::get_frontal_face_detector();
    dlib::deserialize("/home/wessi/catkin_ws/src/dlib_faceshift/src/shape_predictor_68_face_landmarks.dat") >> pose_model;
    
    ros::init(argc, argv, "dlib_faceshift_node");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::String>("blender_values", 1000);
    ros::Subscriber sub = nh.subscribe("/usb_cam_node/image_raw", 1, imageCB);
    ros::spin();
}
