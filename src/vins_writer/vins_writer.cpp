// Write out flat files for VINS - OkVis
// ORBSLAM support removed for now

#include <stdio.h>
#include <inttypes.h>
#include <iostream>
#include <fstream>      // std::ifstream, std::ofstream

#include <boost/filesystem.hpp>

#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core.hpp"
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>
using namespace cv;
using namespace std;

class Pass{
  public:
    Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string image_channel_, std::string ins_channel_,
         std::string output_folder_);
    
    ~Pass(){
      std::cout << "finish\n";

      okvis_cam0_timestamp_file_.close();
      okvis_cam1_timestamp_file_.close();
      okvis_imu0_timestamp_file_.close();
    }    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    bool verbose_;
    std::string image_channel_, ins_channel_;
    std::string output_folder_;
    image_io_utils*  imgutils_;
    uint8_t* img_buf_; 
    uint8_t* rgb_compress_buffer_;

    ofstream okvis_imu0_timestamp_file_;
    ofstream okvis_cam0_timestamp_file_;
    ofstream okvis_cam1_timestamp_file_;
    
    void imagesHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg);

};

Pass::Pass(boost::shared_ptr<lcm::LCM> &lcm_, bool verbose_,
         std::string image_channel_, std::string ins_channel_,
         std::string output_folder_):
    lcm_(lcm_), verbose_(verbose_), 
    image_channel_(image_channel_), ins_channel_(ins_channel_), 
    output_folder_(output_folder_){
  lcm_->subscribe( image_channel_ ,&Pass::imagesHandler,this);
  lcm_->subscribe( ins_channel_ ,&Pass::insHandler,this);

  // left these numbers very large:
  img_buf_= (uint8_t*) malloc(3* 1524  * 1544);
  imgutils_ = new image_io_utils( lcm_, 
                                  1524, 
                                  3*1544 );  

  rgb_compress_buffer_= (uint8_t*) malloc(3* 1524  * 1544);

  boost::filesystem::create_directory(boost::filesystem::path( std::string(output_folder_) ));
  boost::filesystem::create_directory(boost::filesystem::path( std::string(output_folder_ + "/cam0") ));
  boost::filesystem::create_directory(boost::filesystem::path( std::string(output_folder_ + "/cam0/data") ));
  boost::filesystem::create_directory(boost::filesystem::path( std::string(output_folder_ + "/cam1") ));
  boost::filesystem::create_directory(boost::filesystem::path( std::string(output_folder_ + "/cam1/data") ));
  boost::filesystem::create_directory(boost::filesystem::path( std::string(output_folder_ + "/imu0") ));

  std::stringstream ss1;
  ss1 << output_folder_ << "/" << "cam0/data.csv";
  okvis_cam0_timestamp_file_.open (ss1.str().c_str() );
  okvis_cam0_timestamp_file_ << "#timestamp [ns],filename\n";

  std::stringstream ss3;
  ss3 << output_folder_ << "/" << "cam1/data.csv"; 
  okvis_cam1_timestamp_file_.open (ss3.str().c_str() );
  okvis_cam1_timestamp_file_ << "#timestamp [ns],filename\n";

  std::stringstream ss2;
  ss2 << output_folder_ << "/" << "/imu0/data.csv";
  okvis_imu0_timestamp_file_.open (ss2.str().c_str() );
  okvis_imu0_timestamp_file_ << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n";
}

void Pass::insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg){
  std::cout << "got ins" << channel << " " << msg->utime << "\n";

  std::stringstream ss;
  ss.precision(21);
  ss << msg->utime*1E3 << ","
     << msg->gyro[0] << ","
     << msg->gyro[1] << ","
     << msg->gyro[2] << ","
     << msg->accel[0] << ","
     << msg->accel[1] << ","
     << msg->accel[2];

  okvis_imu0_timestamp_file_ << ss.str() << "\n";

}


void Pass::imagesHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg){
  std::cout << "1got " << channel << "\n";

  int w = msg->images[0].width;
  int h = msg->images[0].height;
  
  // left is cam0
  imgutils_->decodeImageToGray( & msg->images[0], img_buf_);
  cv::Mat cam0(cv::Size( w, h),CV_8UC1);
  cam0.data = img_buf_;
  std::stringstream ss;
  ss.precision(21);
  ss << output_folder_ << "/cam0/data/" << msg->utime*1E3 << ".png";
  std::cout << ss.str() << "\n";
  //cv::cvtColor(img, img, CV_RGB2BGR);
  cv::imwrite( ss.str(), cam0);


  // right is cam1
  imgutils_->decodeImageToGray( & msg->images[1], img_buf_);
  cv::Mat cam1(cv::Size( w, h),CV_8UC1);
  cam1.data = img_buf_;
  std::stringstream ss2;
  ss2.precision(21);
  ss2 << output_folder_ << "/cam1/data/" << msg->utime*1E3 << ".png";
  std::cout << ss2.str() << "\n";
  //cv::cvtColor(img, img, CV_RGB2BGR);
  cv::imwrite( ss2.str(), cam1);


  std::stringstream ss3;
  ss3.precision(21);
  ss3 << msg->utime*1E3 << ","
     << msg->utime*1E3 << ".png";
  okvis_cam0_timestamp_file_ << ss3.str() << "\n";
  okvis_cam1_timestamp_file_ << ss3.str() << "\n";
  okvis_cam0_timestamp_file_.flush();
  okvis_cam1_timestamp_file_.flush();

}


int main( int argc, char** argv ){
  ConciseArgs parser(argc, argv, "vins-writer");
  bool verbose=false;
  string image_channel="CAMERA";
  string ins_channel="MICROSTRAIN";
  string output_folder="/tmp/vins";
  parser.add(verbose, "v", "verbose", "Verbosity");
  parser.add(image_channel, "c", "image_channel", "Image channel");
  parser.add(ins_channel, "i", "ins_channel", "INS channel");
  parser.add(output_folder, "o", "output_folder", "Output folder");
  parser.parse();
  cout << verbose << " is verbose\n";
  cout << image_channel << " is image_channel\n";
  cout << output_folder << " is output_folder\n";
  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }


  
  Pass app(lcm,verbose,image_channel, ins_channel, output_folder);
  cout << "Ready to convert from imu to pose" << endl << "============================" << endl;
  while(0 == lcm->handle());

  std::cout << "exit\n";
  return 0;
}
