// task1.cpp

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease){
	if (pInterfaceToRelease != NULL){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	cv::setUseOptimized(true);

	// Sensor
	IKinectSensor* pSensor;
	HRESULT hresult = S_OK;
	hresult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hresult)){
		std::cerr <<"Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hresult = pSensor->Open();
	if (FAILED(hresult)){
		std::cerr <<"Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source
	IColorFrameSource* pColorSource;
	hresult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hresult)){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameSource* pBodySource;
	hresult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hresult)){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hresult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hresult)){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hresult = pBodySource->OpenReader(&pBodyReader);
		if (FAILED(hresult)){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

    // Description
	IFrameDescription* pDescription;
	hresult = pColorSource->get_FrameDescription(&pDescription);
	if (FAILED(hresult)){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width(&width); //1920
	pDescription->get_Height(&height); //1080
	unsigned int bufferSize = width*height * 4 * sizeof(unsigned char);

	cv::Mat bufferMat(height, width, CV_8UC4);
	cv::Mat bodyMat(height / 2, width / 2, CV_8UC4);
	cv::namedWindow("Body");

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b(255, 0, 0);
	color[1] = cv::Vec3b(0, 255, 0);
	color[2] = cv::Vec3b(0, 0, 255);
	color[3] = cv::Vec3b(255, 255, 0);
	color[4] = cv::Vec3b(255, 0, 255);
	color[5] = cv::Vec3b(0, 255, 255);

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hresult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hresult)){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	while (1){

		if (cv::waitKey(2010) == VK_SPACE){
			Sleep(2000);
			std::cout << "Processing Frame" << std::endl;

			// Color Frame to Body Frame Size
			IColorFrame* pColorFrame = nullptr;
			hresult = pColorReader->AcquireLatestFrame(&pColorFrame);
			if (SUCCEEDED(hresult)){
				hresult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
				if (SUCCEEDED(hresult)){
					cv::resize(bufferMat,bodyMat,cv::Size(),0.5,0.5);
				}
			}

			// Body Frame

			IBodyFrame* pBodyFrame = nullptr;
			hresult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
			if (SUCCEEDED(hresult)){
				IBody* pBody[BODY_COUNT] = { 0 };
				hresult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
				if (SUCCEEDED(hresult)){
					for (int count = 0; count < BODY_COUNT; count++){
						BOOLEAN bTracked = false;
						hresult = pBody[count]->get_IsTracked(&bTracked);
						if (SUCCEEDED(hresult) && bTracked){
							std::cout <<"Body "<<count<<" is being tracked"<< std::endl;
							Joint joint[JointType::JointType_Count];
							hresult = pBody[count]->GetJoints(JointType::JointType_Count, joint);
							if (SUCCEEDED(hresult)){
                               // Joint
								for (int type=0; type<JointType::JointType_Count; type++){
									std::cout<<"Joint "<<type;
									std::cout << " X " << joint[type].Position.X;
									std::cout << " Y " << joint[type].Position.Y;
									std::cout << " Z " << joint[type].Position.Z << std::endl;
									ColorSpacePoint colorSpacePoint = { 0 };
									pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);
									if ((x >= 0) && (x<width) && (y >= 0) && (y<height)){
										cv::circle(bufferMat, cv::Point(x, y), 15, static_cast<cv::Scalar>(color[count]), -1, CV_AA);
									}
								}
							}
						}
					}

					cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);

				}
				for (int count = 0; count < BODY_COUNT; count++){
					SafeRelease(pBody[count]);
				}
			}

			SafeRelease(pColorFrame);
			SafeRelease(pBodyFrame);

			cv::imshow("Body", bodyMat);
		}



		if (cv::waitKey(10) == VK_ESCAPE){
			break;
		}

	}

	SafeRelease(pColorSource);
	SafeRelease(pBodySource);
	SafeRelease(pColorReader);
	SafeRelease(pBodyReader);
	SafeRelease(pDescription);
	SafeRelease(pCoordinateMapper);
	if (pSensor){
		pSensor->Close();
	}
	SafeRelease(pSensor);
	cv::destroyAllWindows();

	return 0;
}