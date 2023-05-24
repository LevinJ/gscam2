/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich  
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "yaml-cpp/yaml.h"



#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

struct ocam_model
{
   double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
   int length_pol;                // length of polynomial
   double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
   int length_invpol;             // length of inverse polynomial
   double xc;         // row coordinate of the center
   double yc;         // column coordinate of the center
   double c;          // affine parameter
   double d;          // affine parameter
   double e;          // affine parameter
   int width;         // image width
   int height;        // image height
   
};
struct ex_camera_vehicle
{
   double x;
   double y;
   double z;
   double qw;
   double qx;
   double qy;
   double qz;
};

class ImageStitch
{
   public:
   ImageStitch(){}
   ImageStitch(std::string intrinsics_path, std::string extrinsics_path, std::string vehicle_config_path, double alpha_deg, double beta_deg);
   // ImageStitch();
   ~ImageStitch();
   // int stitch_image ();
   int StichExec(const cv::Mat & rawimg_front,const cv::Mat & rawimg_left,const cv::Mat & rawimg_rear,const cv::Mat & rawimg_right, cv::Mat & sth);
    /*------------------------------------------------------------------------------
   Create undistorted image from extrinsic
   ------------------------------------------------------------------------------*/
   void gen_undist (std::string img_path, std::string intrinsic_path, float sf);
   cv::Mat gen_undist2 (cv::Mat src, std::string intrinsic_path, float sf);

   private:
   std::string intrinsics;
   std::string extrinsics;
   std::string vehicle_config;
   std::string images;
   ocam_model myocam_model;
   std::string vehicle_type;
   float car_width;
   float car_height;
   float car_rear_shaft_mid_2_car_mid;
   int flag = 0;
   float sf = 4;
   
   float stitch_size_height;
   float stitch_size_width;

   float pix_dis;

   int stitch_height;
   int stitch_width;  

   std::vector<cv::Mat> vect_map;
   std::vector<cv::Mat> vect_src;
   std::vector<cv::Mat> vect_mask;
   cv::Mat sth;

   double alpha;
   double beta;
   
   double shaft_height = 0.35;

   ex_camera_vehicle ex_cam2vehicle;
   std::vector<std::string> camera_tags = {"front", "left", "rear", "right"};
   std::vector<std::string> region_tags = {"front", "left", "rear", "right", "top_left", "bottom_left", "bottom_right", "top_right"};


   void Init();
   void Init(std::string intrinsics_path, std::string extrinsics_path, std::string vehicle_config_path, double alpha_deg, double beta_deg);


   /*------------------------------------------------------------------------------
   This functions read the parameters of the omnidirectional camera model and extrinsic 
   from given yaml files
   ------------------------------------------------------------------------------*/
   int get_ocam_model_yaml(std::string filename);
   int get_camera2world_yaml(std::string filename);
   int get_car_config_yaml(std::string filename);
   
   
   /*------------------------------------------------------------------------------
   WORLD2CAM projects a 3D point on to the image
   WORLD2CAM(POINT2D, POINT3D, OCAM_MODEL) 
   projects a 3D point (point3D) on to the image and returns the pixel coordinates (point2D).
   
   POINT3D = [X;Y;Z] are the coordinates of the 3D point.
   OCAM_MODEL is the model of the calibrated camera.
   POINT2D = [rows;cols] are the pixel coordinates of the reprojected point
   
   Copyright (C) 2009 DAVIDE SCARAMUZZA
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

   NOTE: the coordinates of "point2D" and "center" are already according to the C
   convention, that is, start from 0 instead than from 1.
   ------------------------------------------------------------------------------*/
   void world2cam(double point2D[2], double point3D[3], const ocam_model myocam_model);
   
   
   /*------------------------------------------------------------------------------
   CAM2WORLD projects a 2D point onto the unit sphere
   CAM2WORLD(POINT3D, POINT2D, OCAM_MODEL) 
   back-projects a 2D point (point2D), in pixels coordinates, 
   onto the unit sphere returns the normalized coordinates point3D = [x;y;z]
   where (x^2 + y^2 + z^2) = 1.
   
   POINT3D = [X;Y;Z] are the coordinates of the 3D points, such that (x^2 + y^2 + z^2) = 1.
   OCAM_MODEL is the model of the calibrated camera.
   POINT2D = [rows;cols] are the pixel coordinates of the point in pixels
   
   Copyright (C) 2009 DAVIDE SCARAMUZZA   
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

   NOTE: the coordinates of "point2D" and "center" are already according to the C
   convention, that is, start from 0 instead than from 1.
   ------------------------------------------------------------------------------*/
   void cam2world(double point3D[3], double point2D[2], const ocam_model myocam_model);
   
   
   
   /*------------------------------------------------------------------------------
   Create Look Up Table for undistorting the image into a perspective image 
   It assumes the the final image plane is perpendicular to the camera axis
   ------------------------------------------------------------------------------*/
   void create_perspecive_undistortion_LUT( cv::Mat &mapx, cv::Mat &mapy, const struct ocam_model ocam_model, float sf);

   
   // /*------------------------------------------------------------------------------
   // Create Look Up Table for undistorting the image into a panoramic image 
   // It computes a trasformation from cartesian to polar coordinates
   // Therefore it does not need the calibration parameters
   // The region to undistorted in contained between Rmin and Rmax
   // xc, yc are the row and column coordinates of the image center
   // ------------------------------------------------------------------------------*/
   // void create_panoramic_undistortion_LUT ( cv::Mat mapx, cv::Mat mapy, float Rmin, float Rmax, float xc, float yc );

   /*------------------------------------------------------------------------------
   Create initial stitch mask
   ------------------------------------------------------------------------------*/
   // void initial_mask(std::vector<cv::Mat> & vect_mask);
   void initial_mask_alpha_beta(double alpha_in,double beta_in);

   /*------------------------------------------------------------------------------
   Create initial stitch map
   ------------------------------------------------------------------------------*/
   void initial_map();

   /*------------------------------------------------------------------------------
   Create inverse perspective image from extrinsic
   ------------------------------------------------------------------------------*/
   void gen_ipm (std::string img_path, std::string intrinsic_path, std::string extrinsic_path);


   

};









