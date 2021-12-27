#include <opencv2/opencv.hpp>

class Frame {
public:
    Frame(cv::Mat image, float scale = 1.0);
    ~Frame();
    cv::Mat getImage(void) {return _image;}
    cv::Mat getImageWithFeature(void);
    std::vector<cv::Point2f> getKeyPoints(int maxKeyPoints = 500);
    int getFeatureNum(void) {return _keypoint.size();}
    int detectFeatures(int maxFeatureNum);
private:
    int _maxFeatureNum;
    cv::Mat _image, _image_gray;           // image frame(color, gray_scale)
    std::vector<cv::KeyPoint>  _keypoint;  // feature Keypoints
    cv::Mat _descriptor;                   // feature Descriptors
    cv::Point2f _gmv;                      // global motion vector
};