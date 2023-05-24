/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich  
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "bev/image_stitch.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <opencv2/imgproc.hpp>

ImageStitch::ImageStitch(std::string intrinsics_path, std::string extrinsics_path, std::string vehicle_config_path, double alpha_deg=20, double beta_deg=20)
{
  alpha=alpha_deg;
  beta=beta_deg;

  std::cout<<"Stiching INIT "<<std::endl;

  intrinsics = intrinsics_path;
  extrinsics = extrinsics_path;
  vehicle_config = vehicle_config_path;

  ImageStitch::get_car_config_yaml(vehicle_config);

  std::cout<<"intrinsics_path : "<<intrinsics_path<<std::endl;
  std::cout<<"extrinsics_path : "<<extrinsics_path<<std::endl;
  std::cout<<"vehicle_config_path : "<<vehicle_config_path<<std::endl;

  stitch_size_height = 12.0;
  stitch_size_width = 12.0;
  pix_dis = 0.01;
  stitch_height = round(stitch_size_height/pix_dis);
  stitch_width = round(stitch_size_width/pix_dis);
  std::cout<<"stitch_size_height : "<<stitch_size_height<<std::endl;
  std::cout<<"stitch_size_width : "<<stitch_size_width<<std::endl;
  std::cout<<"pix_dis : "<<pix_dis<<std::endl;

  Init();

  std::cout<<"Stiching INIT Done"<<std::endl;

}

ImageStitch::~ImageStitch(){}


void ImageStitch::Init(std::string intrinsics_path, std::string extrinsics_path, std::string vehicle_config_path, double alpha_deg, double beta_deg)
{
  alpha=alpha_deg;
  beta=beta_deg;

  std::cout<<"Stiching INIT "<<std::endl;

  intrinsics = intrinsics_path;
  extrinsics = extrinsics_path;
  vehicle_config = vehicle_config_path;

  std::cout<<"intrinsics_path : "<<intrinsics_path<<std::endl;
  std::cout<<"extrinsics_path : "<<extrinsics_path<<std::endl;
  std::cout<<"vehicle_config_path : "<<vehicle_config_path<<std::endl;

  Init();

  std::cout<<"Stiching INIT Done"<<std::endl;
}

void ImageStitch::Init()
{
  std::cout<<"Loading Vehicle Param "<<std::endl;
  get_car_config_yaml(vehicle_config);
  std::cout<<"Generating Mask "<<std::endl;
  initial_mask_alpha_beta(alpha,beta);
  std::cout<<"Generating Map "<<std::endl;
  initial_map();
}


//------------------------------------------------------------------------------
int ImageStitch::get_ocam_model_yaml(std::string filename)
{
  // char buf[CMV_MAX_BUF];
  int i = 0;
 
  //Open file
  if(!(YAML::LoadFile(filename)))
  {
    std::cout<<"File"+filename+" %s cannot be opened"<<std::endl;		  
    return -1;
  }
  YAML::Node config = YAML::LoadFile(filename);
  
  //Read image size
  myocam_model.width =  config["image size"]["width"].as<int>();
  myocam_model.height =  config["image size"]["height"].as<int>();

  //Read center coordinates -- special attention for x-y!!!
  myocam_model.xc =  config["distortion center"]["cx"].as<double>();
  myocam_model.yc =  config["distortion center"]["cy"].as<double>();

  //Read affine coefficients
  myocam_model.c =  config["affine coefficients"]["c"].as<double>();
  myocam_model.d =  config["affine coefficients"]["d"].as<double>();
  myocam_model.e =  config["affine coefficients"]["e"].as<double>();

  //Read camera2world polynomial coefficients
  myocam_model.length_pol = config["camera2world"]["length_pol"].as<int>();
  for (i = 0; i < myocam_model.length_pol; i++)
    {
        // std::cout << "pol: " << config["camera2world"]["pol"][i].as<double>() << std::endl;
        myocam_model.pol[i] = config["camera2world"]["pol"][i].as<double>();
    }
  //Read world2camera inverse polynomial coefficients
  myocam_model.length_invpol = config["world2camera"]["length_invpol"].as<int>();
  for (i = 0; i < myocam_model.length_invpol; i++)
    {
        myocam_model.invpol[i] = config["world2camera"]["invpol"][i].as<double>();
    }

  /* --------------------------------------------------------------------*/    
  /* Print ocam_model parameters                                         */
  /* --------------------------------------------------------------------*/  
  // printf("pol =\n");    for (i=0; i<myocam_model.length_pol; i++){    printf("\t%e\n",myocam_model.pol[i]); };    printf("\n");
  // printf("invpol =\n"); for (i=0; i<myocam_model.length_invpol; i++){ printf("\t%e\n",myocam_model.invpol[i]); }; printf("\n");  
  // printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",myocam_model.xc,myocam_model.yc,myocam_model.width,myocam_model.height);

  return flag;

}

//------------------------------------------------------------------------------
int ImageStitch::get_camera2world_yaml(std::string filename)
{
  //Open file
  if(!(YAML::LoadFile(filename)))
  {
    std::cout<<"File"+filename+" %s cannot be opened"<<std::endl;			  
    return -1;
  }
  YAML::Node config = YAML::LoadFile(filename);
  
  //Read image size
  ex_cam2vehicle.x =  config["x"].as<double>();
  ex_cam2vehicle.y =  config["y"].as<double>();
  ex_cam2vehicle.z =  config["z"].as<double>() + shaft_height;
  ex_cam2vehicle.qw =  config["qw"].as<double>();
  ex_cam2vehicle.qx =  config["qx"].as<double>();
  ex_cam2vehicle.qy =  config["qy"].as<double>();
  ex_cam2vehicle.qz =  config["qz"].as<double>();

  // std::cout << "extrinsic from camera to vehicle: " << std::endl;
  // std::cout << "x: " << ex_cam2vehicle.x << std::endl;
  // std::cout << "y: " << ex_cam2vehicle.y << std::endl;
  // std::cout << "z: " << ex_cam2vehicle.z << std::endl;
  // std::cout << "qw: " << ex_cam2vehicle.qw << std::endl;
  // std::cout << "qx: " << ex_cam2vehicle.qx << std::endl;
  // std::cout << "qy: " << ex_cam2vehicle.qy << std::endl;
  // std::cout << "qz: " << ex_cam2vehicle.qz << std::endl;

  return flag;
}

//------------------------------------------------------------------------------
int ImageStitch::get_car_config_yaml(std::string filename)
{
  //Open file
  // if(!(YAML::LoadFile(filename)))
  // {
  //   printf("File %s cannot be opened\n", filename);				  
  //   return -1;
  // }
  // YAML::Node config = YAML::LoadFile(filename);
  YAML::Node config;
  try{
         config = YAML::LoadFile(filename);
    }catch(YAML::BadFile &e){
        std::cout<<"read error!"<< filename << std::endl;
        return -1;
    }
  
  //Read vehicle config
  vehicle_type = config["vehicle_type"].as<std::string>();
  car_width =  config["width"].as<float>();
  car_height =  config["length"].as<float>();
  car_rear_shaft_mid_2_car_mid =  config["rear_shaft_mid_2_car_mid"].as<float>();
  
  return flag;
}

//------------------------------------------------------------------------------
void ImageStitch::cam2world(double point3D[3], double point2D[2], const ocam_model myocam_model)
{
 const double *pol    = myocam_model.pol;
 double xc      = (myocam_model.xc);
 double yc      = (myocam_model.yc); 
 double c       = (myocam_model.c);
 double d       = (myocam_model.d);
 double e       = (myocam_model.e);
 int length_pol = (myocam_model.length_pol); 
 double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );
  
 double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 double zp  = pol[0];
 double r_i = 1;
 int i;
 
 for (i = 1; i < length_pol; i++)
 {
   r_i *= r;
   zp  += r_i*pol[i];
 }
 
 //normalize to unit norm
 double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );
 
 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp; 
 point3D[2] = invnorm*zp;
}

//------------------------------------------------------------------------------
void ImageStitch::world2cam(double point2D[2], double point3D[3], const ocam_model myocam_model)
{
 const double *invpol     = myocam_model.invpol; 
 double xc          = (myocam_model.xc);
 double yc          = (myocam_model.yc); 
 double c           = (myocam_model.c);
 double d           = (myocam_model.d);
 double e           = (myocam_model.e);
 int    width       = (myocam_model.width);
 int    height      = (myocam_model.height);
 int length_invpol  = (myocam_model.length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;
  
  if (norm != 0) 
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;
  
    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
//------------------------------------------------------------------------------
void ImageStitch::create_perspecive_undistortion_LUT( cv::Mat &mapx, cv::Mat &mapy, const ocam_model ocam_model, float sf)
{
     int i, j;
     int width = mapx.cols; //New width
     int height = mapx.rows;//New height     
     float Nxc = width/2.0;
     float Nyc = height/2.0;
     float Nz  = -width/sf;
     double M[3];
     double m[2];
     
     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {   
             M[0] = (j - Nxc);
             M[1] = (i - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);
             mapx.at<float>(i, j) = (float) m[0];
             mapy.at<float>(i, j) = (float) m[1];
         }
}


void ImageStitch::initial_mask_alpha_beta(double alpha_in,double beta_in)
{
  vect_mask.clear();
  std::cout<<"alpha,beta "<<alpha_in<<", "<<beta_in<<std::endl;
  cv::Point2f outer_rect_tl(0,0);
  cv::Point2f outer_rect_bl(0,stitch_height);
  cv::Point2f outer_rect_br(stitch_width,stitch_height);
  cv::Point2f outer_rect_tr(stitch_width,0);

  cv::Point2f inner_rect_tl(round(stitch_width/2-car_width/2/pix_dis),round(stitch_height/2-car_height/2/pix_dis));
  cv::Point2f inner_rect_bl(round(stitch_width/2-car_width/2/pix_dis),round(stitch_height/2+car_height/2/pix_dis));
  cv::Point2f inner_rect_br(round(stitch_width/2+car_width/2/pix_dis),round(stitch_height/2+car_height/2/pix_dis));
  cv::Point2f inner_rect_tr(round(stitch_width/2+car_width/2/pix_dis),round(stitch_height/2-car_height/2/pix_dis));

  double l_tan_aplha = (round(stitch_height/2-car_height/2/pix_dis))*tan(alpha_in/180*M_PI);
  double l_tan_beta = (round(stitch_height/2-car_height/2/pix_dis))*tan(beta_in/180*M_PI);


  cv::Point2f tl_1(round(stitch_width/2-car_width/2/pix_dis)-l_tan_aplha,0);
  cv::Point2f tr_1(round(stitch_width/2+car_width/2/pix_dis)+l_tan_aplha,0);
  cv::Point2f bl_1(round(stitch_width/2-car_width/2/pix_dis)-l_tan_beta,stitch_width);
  cv::Point2f br_1(round(stitch_width/2+car_width/2/pix_dis)+l_tan_beta,stitch_width);

  std::vector<cv::Point> vertexes;


  cv::Mat mask_front = cv::Mat::zeros(stitch_width, stitch_height, CV_8UC1);
  
  vertexes.clear();
  vertexes.push_back(tl_1);
  vertexes.push_back(inner_rect_tl);
  vertexes.push_back(inner_rect_tr);
  vertexes.push_back(tr_1);
  cv::fillPoly(mask_front, (std::vector<std::vector<cv::Point>>){vertexes}, cv::Scalar(255,255,255),8,0);//
  vect_mask.push_back(mask_front);

  cv::Mat mask_left = cv::Mat::zeros(stitch_width, stitch_height, CV_8UC1);
  vertexes.clear();
  vertexes.push_back(outer_rect_tl);
  vertexes.push_back(outer_rect_bl);
  vertexes.push_back(bl_1);
  vertexes.push_back(inner_rect_bl);
  vertexes.push_back(inner_rect_tl);
  vertexes.push_back(tl_1);
  cv::fillPoly(mask_left, (std::vector<std::vector<cv::Point>>){vertexes}, cv::Scalar(255,255,255),8,0);//
  vect_mask.push_back(mask_left);


  cv::Mat mask_rear = cv::Mat::zeros(stitch_width, stitch_height, CV_8UC1);
  vertexes.clear();
  vertexes.push_back(inner_rect_bl);
  vertexes.push_back(bl_1);
  vertexes.push_back(br_1);
  vertexes.push_back(inner_rect_br);
  cv::fillPoly(mask_rear, (std::vector<std::vector<cv::Point>>){vertexes}, cv::Scalar(255,255,255),8,0);//
  vect_mask.push_back(mask_rear);

  cv::Mat mask_right = cv::Mat::zeros(stitch_width, stitch_height, CV_8UC1);
  vertexes.clear();
  vertexes.push_back(outer_rect_tr);
  vertexes.push_back(outer_rect_br);
  vertexes.push_back(br_1);
  vertexes.push_back(inner_rect_br);
  vertexes.push_back(inner_rect_tr);
  vertexes.push_back(tr_1);
  cv::fillPoly(mask_right, (std::vector<std::vector<cv::Point>>){vertexes}, cv::Scalar(255,255,255),8,0);//
  vect_mask.push_back(mask_right);
  
}









//------------------------------------------------------------------------------
void ImageStitch::initial_map()
{
  for(int i=0;i<4;i++)
  // for (auto i : camera_tags)
  {

    // std::string img_path = images + i + ".png";
    std::string intr_path = intrinsics + "/fisheye_" + camera_tags[i] + ".yaml";
    std::string extr_path = extrinsics + "/fisheye_" + camera_tags[i]+ "2vehicle.yaml";

    ImageStitch::get_ocam_model_yaml(intr_path);
    ImageStitch::get_camera2world_yaml(extr_path);


    Eigen::Quaterniond qvc(ex_cam2vehicle.qw, ex_cam2vehicle.qx, ex_cam2vehicle.qy, ex_cam2vehicle.qz);
    Eigen::Matrix3d Rvc(qvc);
    Eigen::Vector3d tvc(ex_cam2vehicle.x, ex_cam2vehicle.y, ex_cam2vehicle.z);
    Eigen::Isometry3d Tvc = Eigen::Isometry3d::Identity();
    Tvc.rotate(Rvc);
    Tvc.pretranslate(tvc);
    Eigen::Matrix4d Tcv = Tvc.inverse().matrix();
    Eigen::Matrix3d Rcv = Tcv.block(0,0,3,3);
    Eigen::Vector3d tcv = Tcv.block(0,3,3,1);

    // std::cout << "Tvc: " << Tvc.matrix() << std::endl;
    // std::cout << "Tcv: " << Tcv << std::endl;
    // std::cout << "Rcv: " << Rcv << std::endl;
    // std::cout << "tcv: " << tcv << std::endl;

    // cv::Mat map = cv::Mat::zeros(stitch_height, stitch_width, CV_32FC2);
    cv::Mat ipm = cv::Mat::zeros(stitch_height, stitch_width, CV_8UC3);
    cv::Mat mapx = cv::Mat::zeros(stitch_height, stitch_width, CV_32FC1);
    cv::Mat mapy = cv::Mat::zeros(stitch_height, stitch_width, CV_32FC1);

    const cv::Mat & current_mask=vect_mask[i];
    for (int h = 0; h < stitch_height; h++)
    {
      for (int w = 0; w < stitch_width; w++)
        {

          
          if( current_mask.at<uint8_t>(h,w) <=0)
          {
            //Changhe : This doesn't accelerate stiching, just make the map generation faster
            continue;
          }

          // Eigen::Vector3d Pv(stitch_size_width/2-w*pix_dis, h*pix_dis-stitch_size_height/2+car_rear_shaft_mid_2_car_mid, 0.0); // for old-man-happy only, vehicle cs: xyz-left rear up
          Eigen::Vector3d Pv(-(h*pix_dis-stitch_size_height/2-car_rear_shaft_mid_2_car_mid), stitch_size_width/2-w*pix_dis, 0.0); // for es8 only, vehicle cs: xyz-forward left up
          Eigen::Vector3d Pc = Rcv*Pv + tcv;
          
          double point2D[2];
          double point3D[3];
          point3D[0] = Pc(0); 
          point3D[1] = Pc(1); 
          point3D[2] = -Pc(2);
          ImageStitch::world2cam(point2D, point3D, myocam_model);
          // map.at<cv::Vec2f>(h, w) = (cv::Vec2f) (point2D[0],point2D[1]);
          // map.at<cv::Vec2f>(h, w)[0] = point2D[0];
          // map.at<cv::Vec2f>(h, w)[1] = point2D[1];
          mapx.at<float>(h, w) = (float) point2D[0];
          mapy.at<float>(h, w) = (float) point2D[1];
          
        }
    }
    vect_map.push_back(mapx);
    vect_map.push_back(mapy);
  }
}

// //------------------------------------------------------------------------------
// void create_panoramic_undistortion_LUT ( cv::Mat mapx, cv::Mat mapy, float Rmin, float Rmax, float xc, float yc )
// {
//      int i, j;
//      float theta;
//      int width = mapx.cols;
//      int height = mapx.rows;     
//      float rho;
     
//      for (i=0; i<height; i++)
//          for (j=0; j<width; j++)
//          {
//              theta = -((float)j)/width*2*M_PI; // Note, if you would like to flip the image, just inverte the sign of theta
//              rho   = Rmax - (Rmax-Rmin)/height*i;
//             mapx.at<float>(i, j) = xc + rho*cos(theta); //in OpenCV "x" is the
//             mapy.at<float>(i, j) = yc + rho*sin(theta);                 
//          }
// }

void ImageStitch::gen_undist(std::string img_path, std::string intrinsic_path, float sf)
{
  /* --------------------------------------------------------------------*/
  /* Allocate space for the unistorted images                            */
  /* --------------------------------------------------------------------*/
  cv::Mat src = cv::imread(img_path);

  cv::Mat dst_persp = gen_undist2(src, intrinsic_path, sf);

  /* --------------------------------------------------------------------*/
  /* Display image                                                       */
  /* --------------------------------------------------------------------*/
  cv::namedWindow("Original fisheye camera image", 1);
  cv::imshow("Original fisheye camera image", src);

  cv::namedWindow("Undistorted Perspective Image", 1);
  cv::imshow("Undistorted Perspective Image", dst_persp);

  /* --------------------------------------------------------------------*/
  /* Save image                                                          */
  /* --------------------------------------------------------------------*/
  std::string udimg_path = img_path.substr(0, img_path.size() - 4) + "_undistorted_perspective.png";
  cv::imwrite(udimg_path, dst_persp);
  printf("\nImage %s saved\n", udimg_path.c_str());

  /* --------------------------------------------------------------------*/
  /* Wait until key presses                                              */
  /* --------------------------------------------------------------------*/
  cv::waitKey();
}

cv::Mat ImageStitch::gen_undist2(cv::Mat src, std::string intrinsic_path, float sf)
{
  ImageStitch::get_ocam_model_yaml(intrinsic_path);
  cv::Mat dst_persp = cv::Mat::zeros(src.size(), CV_8UC3);

  cv::Mat mapx_persp = cv::Mat::zeros(src.rows, src.cols, CV_32FC1);
  cv::Mat mapy_persp = cv::Mat::zeros(src.rows, src.cols, CV_32FC1);

  /* --------------------------------------------------------------------  */
  /* Create Look-Up-Table for perspective undistortion                     */
  /* SF is kind of distance from the undistorted image to the camera       */
  /* (it is not meters, it is just a zoom fator)                            */
  /* Try to change SF to see how it affects the result                     */
  /* The undistortion is done on a  plane perpendicular to the camera axis */
  /* --------------------------------------------------------------------  */

  ImageStitch::create_perspecive_undistortion_LUT(mapx_persp, mapy_persp, myocam_model, sf);

  /* --------------------------------------------------------------------*/
  /* Undistort using specified interpolation method                      */
  /* Other possible values are (see OpenCV doc):                         */
  /* CV_INTER_NN - nearest-neighbor interpolation,                       */
  /* CV_INTER_LINEAR - bilinear interpolation (used by default)          */
  /* CV_INTER_AREA - resampling using pixel area relation. It is the preferred method for image decimation that gives more-free results. In case of zooming it is similar to CV_INTER_NN method. */
  /* CV_INTER_CUBIC - bicubic interpolation.                             */
  /* --------------------------------------------------------------------*/
  cv::remap(src, dst_persp, mapx_persp, mapy_persp, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  // cv::INTER_LINEAR+cv::WARP_FILL_OUTLIERS
  return dst_persp;
}

void ImageStitch::gen_ipm (std::string img_path, std::string intrinsic_path, std::string extrinsic_path)
{
  cv::Mat src = cv::imread(img_path);
  ImageStitch::get_ocam_model_yaml(intrinsic_path);
  ImageStitch::get_camera2world_yaml(extrinsic_path);
  Eigen::Quaterniond qvc(ex_cam2vehicle.qw, ex_cam2vehicle.qx, ex_cam2vehicle.qy, ex_cam2vehicle.qz);
  Eigen::Matrix3d Rvc(qvc);
  Eigen::Vector3d tvc(ex_cam2vehicle.x, ex_cam2vehicle.y, ex_cam2vehicle.z);
  Eigen::Isometry3d Tvc = Eigen::Isometry3d::Identity();
  Tvc.rotate(Rvc);
  Tvc.pretranslate(tvc);
  Eigen::Matrix4d Tcv = Tvc.inverse().matrix();
  Eigen::Matrix3d Rcv = Tcv.block(0,0,3,3);
  Eigen::Vector3d tcv = Tcv.block(0,3,3,1);

  // std::cout << "Tvc: " << Tvc.matrix() << std::endl;
  // std::cout << "Tcv: " << Tcv << std::endl;
  // std::cout << "Rcv: " << Rcv << std::endl;
  // std::cout << "tcv: " << tcv << std::endl;

  cv::Mat mapx = cv::Mat::zeros(stitch_height, stitch_width, CV_32FC1);
  cv::Mat mapy = cv::Mat::zeros(stitch_height, stitch_width, CV_32FC1);
  cv::Mat origin_mask = cv::Mat::zeros(stitch_height, stitch_width, CV_8UC1);



  cv::Mat ipm = cv::Mat::zeros(stitch_height, stitch_width, CV_8UC3);
  for (int h = 0; h < stitch_height; h++)
  {
    for (int w = 0; w < stitch_width; w++)
      {
        // Eigen::Vector3d Pv(stitch_size_width/2-w*pix_dis, h*pix_dis-stitch_size_height/2+car_rear_shaft_mid_2_car_mid, 0.0); // for old-man-happy only, vehicle cs: xyz-left rear up
        Eigen::Vector3d Pv(-(h*pix_dis-stitch_size_height/2-car_rear_shaft_mid_2_car_mid), stitch_size_width/2-w*pix_dis, 0.0); // for es8 only, vehicle cs: xyz-forward left up
        Eigen::Vector3d Pc = Rcv*Pv + tcv;
        
        double point2D[2];
        double point3D[3];
        point3D[0] = Pc(0); 
        point3D[1] = Pc(1); 
        point3D[2] = -Pc(2);
        ImageStitch::world2cam(point2D, point3D, myocam_model);
        mapx.at<float>(h, w) = (float) point2D[0];
        mapy.at<float>(h, w) = (float) point2D[1];
        if (point2D[0] < 0 || point2D[0] > src.cols-1 || point2D[1] < 0 || point2D[1] > src.rows-1)
          origin_mask.at<u_char>(h,w) = 0;
        else
          origin_mask.at<u_char>(h,w) = 255;
      }
  }
  cv::remap( src, ipm, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) ); 
  std::string ipm_path = img_path.substr(0, img_path.size()-4)+ "_ipm.png";
  cv::imwrite(ipm_path, ipm);

}

int ImageStitch::StichExec(const cv::Mat & rawimg_front,const cv::Mat & rawimg_left,const cv::Mat & rawimg_rear,const cv::Mat & rawimg_right, cv::Mat & sth)
{
  cv::Mat tmp;
  sth = cv::Mat::zeros(stitch_height, stitch_width, CV_8UC3);

  std::vector<cv::Mat> src={rawimg_front,rawimg_left,rawimg_rear,rawimg_right};

  for(int i = 0; i < 4; i++)
  {
      cv::remap( src[i], tmp, vect_map[2*i], vect_map[2*i+1], cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) ); 
      tmp.copyTo(sth, vect_mask[i]);
      // cv::imshow("test_sth",sth);
      // cv::waitKey();
  }
  return 1;
}
