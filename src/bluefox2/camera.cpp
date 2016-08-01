#include "camera.h"
#include <boost/format.hpp>
#include <algorithm>

namespace bluefox2 {

Camera::Camera(ros::NodeHandle _param_nh) : pnode(_param_nh) {
    int exposure_time_us;
    bool use_color;
    bool use_hdr;
    bool has_hdr;
    bool use_binning;
    bool use_auto_exposure;
    int auto_speed;
    double fps;
    double gain;
    bool is_slave;

    pnode.param("use_color", use_color, false);
    pnode.param("use_hdr", use_hdr, true);
    pnode.param("has_hdr", has_hdr, true);
    pnode.param("use_binning", use_binning, false);
    pnode.param("use_auto_exposure", use_auto_exposure, false);
    pnode.param("exposure_time_us", exposure_time_us, 10000);
    pnode.param("auto_speed", auto_speed, 0);
    pnode.param("fps", fps, 30.0);
    pnode.param("gain", gain, 0.0);
    pnode.param("is_slave", is_slave, false);

    pnode.param("cam_cnt", cam_cnt, 1);

    ROS_ASSERT_MSG(cam_cnt <= MAX_CAM_CNT, "cam_cnt exceeds MAX_CAM_CNT(%d)", MAX_CAM_CNT);

    for (int i = 0; i < cam_cnt; i++) {
        std::string prefix = boost::str(boost::format("camera%d/") % i);

        CameraSetting cs;
        pnode.param(prefix + "serial", cs.serial, std::string(""));
        pnode.param(prefix + "topic", cs.topic, boost::str(boost::format("~%c") % ('a' + i)));

        // pnode.param(prefix + std::string("fps"), fps, 30.0);
        pnode.param(prefix + "use_auto_exposure", cs.use_auto_exposure, use_auto_exposure);
        pnode.param(prefix + "exposure_time_us", cs.exposure_time_us, exposure_time_us);
        pnode.param(prefix + "auto_speed", cs.auto_speed, auto_speed);
        pnode.param(prefix + "gain", cs.gain, gain);
        pnode.param(prefix + "is_slave", cs.is_slave, is_slave);

        ROS_ASSERT(cs.auto_speed < 3);

        cameraSettings[cs.serial] = cs;
    }

    for (auto& item : cameraSettings) {
        const CameraSetting& cs = item.second;
        img_publisher[cs.serial] = node.advertise<sensor_msgs::Image>(cs.topic, 10);
        img_buffer[cs.serial] = sensor_msgs::Image();
    }

    // Count cameras
    devCnt = devMgr.deviceCount();
    ROS_INFO("All %d mvBlueFOX devices", devCnt);

    // Init cameras
    ok = true;
    for (auto& item : cameraSettings) {
        CameraSetting& cs = item.second;
        for (unsigned int k = 0; k < devCnt; k++) {
            if (devMgr[k]->serial.read() == cs.serial) {
                ids[cs.serial] = k;
                if (!initSingleMVDevice(k, cs)) {
                    ok = false;
                    ROS_ERROR("Camera %s Init Failed.", cs.serial);
                    break;
                }
            }
        }
    }
}

Camera::~Camera() {
    for (auto& item : cameraSettings) {
        const int devIndex = ids.at(item.second.serial);
        fi[devIndex]->imageRequestReset(0, 0);
        devMgr[devIndex]->close();
    }
    ok = false;
}

bool Camera::isOK() {
    return ok;
}

bool Camera::initSingleMVDevice(unsigned int id, const CameraSetting& cs) {
    ROS_INFO("Camera Found:  %s(%s)",
             devMgr[id]->family.read().c_str(),
             devMgr[id]->serial.read().c_str());

    try {
        devMgr[id]->open();
    } catch (const mvIMPACT::acquire::ImpactAcquireException& e) {
        std::cout << "An error occurred while opening the device " << devMgr[id]->serial.read()
                  << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << "))."
                  << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    try {
        fi[id] = new mvIMPACT::acquire::FunctionInterface(devMgr[id]);
    } catch (const mvIMPACT::acquire::ImpactAcquireException& e) {
        std::cout << "An error occurred while creating the function interface on device "
                  << devMgr[id]->serial.read() << "(error code: " << e.getErrorCode() << "("
                  << e.getErrorCodeAsString() << "))." << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    try {
        statistics[id] = new mvIMPACT::acquire::Statistics(devMgr[id]);
    } catch (const mvIMPACT::acquire::ImpactAcquireException& e) {
        std::cout << "An error occurred while initializing the statistical information on device "
                  << devMgr[id]->serial.read() << "(error code: " << e.getErrorCode() << "("
                  << e.getErrorCodeAsString() << "))." << std::endl
                  << "Press [ENTER] to end the application..." << std::endl;
        return false;
    }

    // Set Properties
    mvIMPACT::acquire::SettingsBlueFOX settings(devMgr[id]);  // Using the "Base" settings (default)

    // Binning
    if (cs.use_binning) {
        settings.cameraSetting.binningMode.write(cbmBinningHV);
        ROS_INFO("2X Binning");
    } else {
        ROS_INFO("No Binning");
    }

    // Gain
    settings.cameraSetting.autoGainControl.write(agcOff);
    if (cs.gain >= 0.0) {
        settings.cameraSetting.gain_dB.write(cs.gain);
        ROS_INFO("Gain:  %f", settings.cameraSetting.gain_dB.read());
    } else {
        settings.cameraSetting.autoGainControl.write(agcOn);
        ROS_INFO("Auto Gain %d", settings.cameraSetting.autoGainControl.read());
    }

    // Auto exposure, modified controller for better results, be careful about the minimum exposure
    // time
    if (use_auto_exposure) {
        // settings.cameraSetting.autoControlParameters.controllerSpeed.write(acsUserDefined);
        // settings.cameraSetting.autoControlParameters.controllerGain.write(0.5);
        // settings.cameraSetting.autoControlParameters.controllerIntegralTime_ms.write(100);
        // settings.cameraSetting.autoControlParameters.controllerDerivativeTime_ms.write(0.0001);
        settings.cameraSetting.autoControlParameters.controllerSpeed.write(cs.auto_speed);
        settings.cameraSetting.autoControlParameters.desiredAverageGreyValue.write(100);
        settings.cameraSetting.autoControlParameters.controllerDelay_Images.write(0);
        settings.cameraSetting.autoControlParameters.exposeLowerLimit_us.write(50);
        settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.write(cs.exposure_time_us);
        settings.cameraSetting.autoExposeControl.write(aecOn);
        ROS_INFO("Auto Exposure w/ Max Exposure Time (us) :  %d . Desired Grey Value: %d",
                 settings.cameraSetting.autoControlParameters.exposeUpperLimit_us.read(),
                 settings.cameraSetting.autoControlParameters.desiredAverageGreyValue.read());
    } else {
        settings.cameraSetting.autoExposeControl.write(aecOff);
        settings.cameraSetting.expose_us.write(cs.exposure_time_us);
        ROS_INFO("Exposure Time (us) :  %d", settings.cameraSetting.expose_us.read());
    }

    // HDR
    if (cs.has_hdr) {
        if (cs.use_hdr) {
            settings.cameraSetting.getHDRControl().HDRMode.write(cHDRmFixed0);
            settings.cameraSetting.getHDRControl().HDREnable.write(bTrue);
            ROS_INFO("Enable HDR ...");
            ROS_INFO("KneePoint 0:");
            ROS_INFO("  Voltage (mV):      %d",
                     settings.cameraSetting.getHDRControl()
                         .getHDRKneePoint(0)
                         .HDRControlVoltage_mV.read());
            ROS_INFO(
                "  Parts per Million: %d",
                settings.cameraSetting.getHDRControl().getHDRKneePoint(0).HDRExposure_ppm.read());
            ROS_INFO("KneePoint 1:");
            ROS_INFO("  Voltage (mV):      %d",
                     settings.cameraSetting.getHDRControl()
                         .getHDRKneePoint(1)
                         .HDRControlVoltage_mV.read());
            ROS_INFO(
                "  Parts per Million: %d",
                settings.cameraSetting.getHDRControl().getHDRKneePoint(1).HDRExposure_ppm.read());
        } else {
            settings.cameraSetting.getHDRControl().HDREnable.write(bFalse);
            ROS_INFO("HDR Off");
        }
    } else {
        ROS_INFO("No HDR");
    }

    // Color
    if (cs.use_color) {
        // RGB image
        settings.imageDestination.pixelFormat.write(idpfBGR888Packed);
        ROS_INFO("Color Images");
    } else {
        // Raw image
        settings.imageDestination.pixelFormat.write(idpfRaw);
        ROS_INFO("Grayscale/Bayer Images");
    }

    // prefill the capture queue. There can be more then 1 queue for some device, but only one for
    // now
    mvIMPACT::acquire::SystemSettings ss(devMgr[id]);
    ss.requestCount.write(1);

    // Trigger setting
    if (cs.is_slave) {
        ROS_INFO("Set Master Camera\n");
        // settings.cameraSetting.triggerMode.write(ctmOnDemand);
        settings.cameraSetting.flashMode.write(cfmDigout0);
        settings.cameraSetting.flashType.write(cftStandard);
        settings.cameraSetting.flashToExposeDelay_us.write(0);
    } else  // Slave camera
        ROS_INFO("Set Slave Camera\n");
    settings.cameraSetting.triggerMode.write(ctmOnHighLevel);
    settings.cameraSetting.triggerSource.write(ctsDigIn0);
    settings.cameraSetting.frameDelay_us.write(0);
}

return true;
}

void Camera::feedImages() {
    ros::Rate r(fps);
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);
    sensor_msgs::ImagePtr left(new sensor_msgs::Image);
    sensor_msgs::ImagePtr right(new sensor_msgs::Image);
    while (pnode.ok()) {
        if (grab_image_data()) {
            for (int i = 0; i < pub_cnt; i++) {
                img_buf[i].header.stamp = capture_time;
                img_buf[i].header.frame_id = std::string("image");
                pub_img[i].publish(img_buf[i]);
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}

bool Camera::grab_image_data() {
    // Request images from both cameras
    for (auto& item : ids) {
        const int devIndex = item.second;
        fi[devIndex]->imageRequestSingle();
    }
    usleep(10000);  // necessary short sleep to warm up the camera
    capture_time = ros::Time::now();

    int requestNr[MAX_CAM_CNT] = {INVALID_ID};

    for (auto& item : ids) {
        const int devIndex = item.second;
        requestNr[devIndex] = fi[devIndex]->imageRequestWaitFor(300);
    }

    bool status = true;
    for (auto& item : ids) {
        const int devIndex = item.second;
        if (!(fi[devIndex]->isRequestNrValid(requestNr[devIndex]))) {
            status = false;
            ROS_ERROR("Camera %s request timeout.", item.first.c_str());
        }
    }

    if (status) {
        int ok_cnt = 0;
        for (auto& item : ids) {
            const int devIndex = item.second;
            pRequest[devIndex] = fi[devIndex]->getRequest(requestNr[devIndex]);
            ok_cnt += pRequest[devIndex]->isOK();

            if (!pRequest[devIndex]->isOK()) {
                ROS_ERROR("Camera %s request failed.", item.first.c_str());
            }
        }

        if (ok_cnt == cam_cnt) {
            for (auto& item : ids) {
                const int devIndex = ids.begin()->second;
                const std::string& serial = ids.begin()->first;

                unsigned int Channels = pRequest[ids[0]]->imageChannelCount.read();
                unsigned int Height = pRequest[ids[0]]->imageHeight.read();
                unsigned int Width = pRequest[ids[0]]->imageWidth.read();
                unsigned int Step = Width * Channels;
                unsigned int Data_cnt = Height * Step;
                std::string Encoding = Channels == 1 ? sensor_msgs::image_encodings::MONO8
                                                     : sensor_msgs::image_encodings::BGR8;

                img_buffer.at(serial).height = Height;
                img_buffer.at(serial).width = Width;
                img_buffer.at(serial).step = Step;
                img_buffer.at(serial).encoding = Encoding;

                img_buffer.at(serial).data.resize(Step * Height);
                std::copy(pRequest[z[k]]->imageData.read(),
                          pRequest[z[k]]->imageData.read() + Data_cnt,
                          &img_buffer.at(serial).data);
            }

            // Release capture request
            for (auto& item : ids) {
                const int devIndex = ids.begin()->second;
                fi[devIndex]->imageRequestUnlock(requestNr[devIndex]);
            }

            status = true;

        } else {
            ROS_ERROR("Not all requests are vaild, discard all data. (2)");
            // Clear all image received and reset capture
            for (auto& item : ids) {
                const int devIndex = ids.begin()->second;
                fi[devIndex]->imageRequestUnlock(requestNr[devIndex]);
            }

            status = false;
        }
    } else {
        ROS_ERROR("Not all requests are vaild, discard all data. (1)");
        // Clear all image received and reset capture
        for (auto& item : ids) {
            const int devIndex = ids.begin()->second;
            if (fi[devIndex]->isRequestNrValid(requestNr[devIndex])) {
                pRequest[devIndex] = fi[devIndex]->getRequest(requestNr[devIndex]);
                fi[devIndex]->imageRequestUnlock(requestNr[devIndex]);
            }
        }
        status = false;
    }
    return status;
}
