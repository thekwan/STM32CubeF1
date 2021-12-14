#include <opencv2/opencv.hpp>

class Frame {
public:
    Frame(cv::Mat image, bool resize = false, int maxFeatureNum = 2000);
    ~Frame();
    cv::Mat getImage(void) {return _image;}
    cv::Mat getImageFeature(void);
    int getFeatureNum(void) {return _keypoint.size();}
private:
    int _maxFeatureNum;
    cv::Mat _image, _image_gray;           // image frame(color, gray_scale)
    std::vector<cv::KeyPoint>  _keypoint;  // feature Keypoints
    cv::Mat _descriptor;                   // feature Descriptors
    cv::Point2f _gmv;                      // global motion vector
};
