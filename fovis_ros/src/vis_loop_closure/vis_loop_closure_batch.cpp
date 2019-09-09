// test program:
/// given a set of images and visual features
// choose a reference and see if you can match all the others to it
//
// matching: finds inlier matches and then estimate the transformation between the images
//
// match 0050 from snippet 3 to all the images in snippet_2
// drc-vis-loop-closure-batch  -P husky/robot.cfg   -f snippet_2/  -r snippet_3/0050_1465298302290122
// Result: matches to images 43, 49, 50, 51 in snipper 2
//
//
// July 2016

#include <dirent.h>
#include <algorithm> // std::sort, std::copy

#include "vis_loop_closure/vis_loop_closure.hpp"
#include <ConciseArgs>
#include <path_util/path_util.h>

using namespace std;

class RegApp{
  public:
    RegApp(boost::shared_ptr<lcm::LCM> &publish_lcm, VisLoopClosureConfig reg_cfg);
    
    ~RegApp(){
    }
    
    void doVisLoopClosureBatch(std::string path_to_folder, std::string ref_filename);
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;

    VisLoopClosure::Ptr reg;
};

struct FrameMatchResult{
  FrameMatchPtr match;
  int counter;
  std::string name;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef boost::shared_ptr<FrameMatchResult> FrameMatchResultPtr;


RegApp::RegApp(boost::shared_ptr<lcm::LCM> &lcm_, VisLoopClosureConfig reg_cfg):          
    lcm_(lcm_){
  reg = VisLoopClosure::Ptr (new VisLoopClosure (lcm_, reg_cfg));
}
  

void RegApp::doVisLoopClosureBatch(std::string path_to_folder, std::string ref_filename){

  std::vector<string> futimes;
  std::vector<string> utimes_strings;
  reg->getFilenames(path_to_folder, futimes, utimes_strings);

  
  istringstream temp_buffer( ref_filename );
  string fname0;
  temp_buffer >> fname0; 
  
  string temp = fname0.substr(5,16);
  istringstream temp_buffer2( temp);
  int64_t utime0;
  temp_buffer2 >> utime0 ; 
  
  cout << fname0 << " fname\n";
  cout << utime0 << " utime0\n";
  
  stringstream ifile0, featfile0;
  ifile0 << fname0 << "_left.png";
  featfile0 << fname0 << ".feat";
  cv::Mat img0 = cv::imread( ifile0.str(), CV_LOAD_IMAGE_GRAYSCALE );
  std::vector<ImageFeature> features0;
  reg->read_features(featfile0.str(), features0);

  std::vector<FrameMatchResultPtr> match_results;
  
  for (size_t i= 0 ; i<  futimes.size(); i++){
    cout << "\n";
    istringstream buffer(utimes_strings[i]);
    int64_t utime1;
    buffer >> utime1; 

    cout << i << ": ";
    cout << fname0 << " to "<<futimes[i] <<"\n";
    stringstream ifile1, featfile1;
    ifile1 << path_to_folder << "/" << futimes[i] << "_left.png";
    featfile1 << path_to_folder << "/" << futimes[i] << ".feat";

    /// 1. Read in imgs and featues:
    cv::Mat img1 = cv::imread( ifile1.str(), CV_LOAD_IMAGE_GRAYSCALE );
  
    std::vector<ImageFeature> features1;
    reg->read_features(featfile1.str(), features1);
    
    FrameMatchPtr match(new FrameMatch());
    reg->align_images(img0, img1, features0, features1, utime0, utime1,match );

    if (match->status == pose_estimator::SUCCESS){
      Eigen::Quaterniond delta_quat = Eigen::Quaterniond(match->delta.rotation());
      cout << match->delta.translation().x() << " "
        << match->delta.translation().y() << " "
        << match->delta.translation().z() << " | "
        << delta_quat.w() << " "
        << delta_quat.x() << " "
        << delta_quat.y() << " "
        << delta_quat.z() << " "
        << " (success)\n";

      FrameMatchResultPtr match_result(new FrameMatchResult());
      match_result->match = match;
      match_result->counter = i;
      match_result->name = futimes[i];

      match_results.push_back(match_result);
    }else{
      std::cout << "Failed Match\n";
    }
  }

  for (size_t i=0; i<match_results.size(); i++)
    std::cout << match_results[i]->counter << " "
              << match_results[i]->name << " "
              << match_results[i]->match->n_registration_inliers << " "
              << match_results[i]->match->status << "\n";


  std::ofstream output_file;

  std::stringstream output_filename;
  output_filename << fname0 << "_reg_output.txt";
  std::cout << output_filename.str() << " is output filename\n";
  output_file.open(output_filename.str().c_str());
  output_file << "# no utime n_registration_inliers status\n";

  for (size_t i=0; i<match_results.size(); i++){
    std::string s = match_results[i]->name;
    std::replace( s.begin(), s.end(), '_', ' '); // replace all '_' to ' '
    output_file << s << " "
                << match_results[i]->match->n_registration_inliers << " "
                << match_results[i]->match->status << "\n";
  }
  output_file.flush();
  output_file.close();

}


int 
main( int argc, char** argv ){
  VisLoopClosureConfig reg_cfg;
  reg_cfg.min_inliers = 60; // 60 used by Hordur, might want to use a higher number
  reg_cfg.verbose = FALSE;
  reg_cfg.publish_diagnostics = FALSE;
  reg_cfg.use_cv_show = FALSE;
  std::string param_file = ""; // actual file
  reg_cfg.param_file = ""; // full path to file

  string path_to_folder = ".";
  string reference="0050_1465297459081387";

  ConciseArgs parser(argc, argv, "registration-batch");
  parser.add(reg_cfg.min_inliers, "i", "min_inliers", "Min number of inliers");
  parser.add(reg_cfg.verbose, "v", "verbose", "Verbose");
  parser.add(reg_cfg.publish_diagnostics, "d", "publish_diagnostics", "Publish LCM diagnostics");
  parser.add(reg_cfg.use_cv_show, "c", "use_cv_show", "Use opencv show");
  parser.add(path_to_folder, "f", "path_to_folder", "path_to_folder continaing test images");
  parser.add(reference, "r", "reference", "Reference filename root");
  parser.add(param_file, "P", "param_file", "Pull params from this file instead of LCM");
  parser.parse();

  cout << reference << " is reference\n"; 
  
  reg_cfg.param_file = std::string(getConfigPath()) +'/' + std::string(param_file);
  if (param_file.empty()) { // get param from lcm
    reg_cfg.param_file = "";
  }

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  RegApp app(lcm, reg_cfg);
  
  app.doVisLoopClosureBatch(path_to_folder, reference);
  cout << "registration batch is done" << endl << endl;

  return 0;
}
