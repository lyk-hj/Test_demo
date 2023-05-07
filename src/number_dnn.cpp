#include "number_dnn.h"

using namespace cv;
using namespace std;

//namespace robot_detection {
DNN_detect::DNN_detect()
{
    FileStorage fs("../other/dnn_data.yaml", FileStorage::READ);
    net_path = (string)fs["net_path"];
    input_width = (int)fs["input_width"];
    input_height = (int)fs["input_height"];
    net = dnn::readNetFromONNX(net_path);
//    net.setPreferableTarget(dnn::dnn4_v20211004::DNN_TARGET_CUDA_FP16);
//    net.setPreferableBackend(dnn::dnn4_v20211004::DNN_BACKEND_CUDA);
_mean = Mat(Size(input_width, input_height), CV_32FC3,Scalar(0.01505453476347024, 0.01179885564375693, 0.0142095491861419));
_std = Mat(Size(input_width, input_height), CV_32FC3, Scalar(0.01424329199190097, 0.029395047427766323, 0.04076346776268015));
    fs.release();
}

void DNN_detect::img_processing(Mat ori_img, std::vector<cv::Mat>& numROIs, std::vector<cv::Mat>& numBlobs) {
    Mat out_blob;
//    cv::waitKey(0);
//    cvtColor(ori_img, ori_img, cv::COLOR_RGB2GRAY);
    threshold(ori_img, ori_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
//    cv::resize(ori_img,ori_img,cv::Size(input_width,input_height));
//    cv::imshow("numROI",ori_img);
//    ori_img.convertTo(ori_img,CV_32FC3, 1.0f/255.0f);
//    subtract(ori_img, _mean, ori_img);
//    divide(ori_img, _std, out_blob);
//    std::cout<<out_blob.at<float>(10,10, 0)<<std::endl;
    numROIs.push_back(ori_img);
    numBlobs.push_back(ori_img);
}

Mat DNN_detect::net_forward(const std::vector<cv::Mat>& numBlobs) {
    //!< opencv dnn supports dynamic batch_size, later modify
    cv::Mat blob;
    dnn::blobFromImages(numBlobs, blob, 1.f/255.f, Size(input_width, input_height));
//    std::cout<<blob.size<<std::endl;
    net.setInput(blob);
//    std::cout<<net.getLayerInputs(0);
//    std::cout<<net.getLayerId("/max_pool_2/MaxPool_output_0")<<std::endl;
//    Mat ou = net.forward("/dense/dense.4/MatMul_output_0");
//    std::cout<<ou<<std::endl;
    Mat outputs = net.forward();
//    std::cout<<outputs.size<<std::endl;
    return outputs;
}

//}