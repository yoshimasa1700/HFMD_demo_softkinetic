////////////////////////////////////////////////////////////////////////////////
// SoftKinetic DepthSense SDK
//
// COPYRIGHT AND CONFIDENTIALITY NOTICE - SOFTKINETIC CONFIDENTIAL
// INFORMATION
//
// All rights reserved to SOFTKINETIC SENSORS NV (a
// company incorporated and existing under the laws of Belgium, with
// its principal place of business at Boulevard de la Plainelaan 15,
// 1050 Brussels (Belgium), registered with the Crossroads bank for
// enterprises under company number 0811 341 454 - "Softkinetic
// Sensors").
//
// The source code of the SoftKinetic DepthSense Camera Drivers is
// proprietary and confidential information of Softkinetic Sensors NV.
//
// For any question about terms and conditions, please contact:
// info@softkinetic.com Copyright (c) 2002-2012 Softkinetic Sensors NV
////////////////////////////////////////////////////////////////////////////////

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <vector>
#include <exception>
#include <string>

#include <DepthSense.hxx>

#include <opencv2/opencv.hpp>

#include <HFMD_core/CRForest.h>
#include <HFMD_core/util.h>
#include <HFMD_core/CDataset.h>

#include <gflags/gflags.h>

#include "./CCalibDS325.h"

using namespace DepthSense;
using namespace std;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;

int imageNum = 0;

cv::Mat g_depth,g_color;

CRForest *g_forest;

CCalibDS325 *g_calib;


/*----------------------------------------------------------------------------*/
// New audio sample event handler
// void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
// {
//     printf("A#%u: %d\n",g_aFrames,data.audioData.size());
//     g_aFrames++;
// }

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data){
  printf("C#%u: %d\n",g_cFrames,data.colorMap.size());
  //*data.colorMap;
  //data.colorMap

  // printf("C2#: %d\n", g_color.cols * g_color.rows * g_color.channels());
  // printf("C3: %d\n", sizeof(*g_color.data));

  memcpy(g_color.data, data.colorMap, data.colorMap.size());

  
  int key = cv::waitKey(1);

  CTestDataset seqImg;

  cv::Mat scaledDepth;//(g_depth.rows * 2, g_depth.cols * 2, CV_16UC1);

  //cv::resize(g_depth, scaledDepth, scaledDepth.size());

  g_calib->calib(g_color, g_depth, g_color, scaledDepth);

  std::cout << g_depth.size() << std::endl;
  seqImg.img.push_back(&g_color);
  seqImg.img.push_back(&scaledDepth);

  // cv::Mat maxDist = cv::Mat::ones(g_color.rows , g_color.cols , CV_16UC1) * MAX_DEPTH;
  // cv::Mat minDist = cv::Mat::ones(g_color.rows , g_color.cols , CV_16UC1) * MIN_DEPTH;
  // cv::resize(g_depth, scaledDepth, cv::Size(), 2.0,2.0);
  // cv::min(scaledDepth, maxDist, scaledDepth);
  // scaledDepth -= minDist;
  // scaledDepth.convertTo(scaledDepth, CV_8UC1, 255.0 / (MAX_DEPTH - MIN_DEPTH));

  

  CDetectionResult detectR;

  detectR = g_forest->detection(seqImg);

  std::cout << g_depth.size() << std::endl;

  if(key == 't'){

    stringstream ss_c, ss_d;
    ss_c << "color_" << imageNum << ".png";
    ss_d << "depth_" << imageNum << ".png";

    cv::imwrite(ss_c.str(), g_color);
    cv::imwrite(ss_d.str(), g_depth);

    imageNum++;
  }else if(key == 'q'){
    g_context.quit();    
  }

  cv::Mat showDepth;
  scaledDepth.convertTo(showDepth, CV_8UC1, 255.0 / (MAX_DEPTH));

  cv::circle(showDepth, cv::Point(320, 240), 5, cv::Scalar(0,0,0), 5);


  cv::imshow("color", g_color);
  cv::imshow("depth", showDepth);
  

  key = cv::waitKey(1);

  g_cFrames++;
}

/*----------------------------------------------------------------------------*/
// New depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
  printf("Z#%u: %d\n",g_dFrames,data.vertices.size());

  // // Project some 3D points in the Color Frame
  // if (!g_pProjHelper)
  // {
  //     g_pProjHelper = new ProjectionHelper (data.stereoCameraParameters);
  //     g_scp = data.stereoCameraParameters;
  // }
  // else if (g_scp != data.stereoCameraParameters)
  // {
  //     g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
  //     g_scp = data.stereoCameraParameters;
  // }

  // int32_t w, h;
  // FrameFormat_toResolution(data.captureConfiguration.frameFormat,&w,&h);
  // int cx = w/2;
  // int cy = h/2;

  // printf("%d, %d\n", w, h);
  // printf("%d", data.depthMap.size());

  // Vertex p3DPoints[4];

  // p3DPoints[0] = data.vertices[(cy-h/4)*w+cx-w/4];
  // p3DPoints[1] = data.vertices[(cy-h/4)*w+cx+w/4];
  // p3DPoints[2] = data.vertices[(cy+h/4)*w+cx+w/4];
  // p3DPoints[3] = data.vertices[(cy+h/4)*w+cx-w/4];
    
  // Point2D p2DPoints[4];
  // g_pProjHelper->get2DCoordinates ( p3DPoints, p2DPoints, 4, CAMERA_PLANE_COLOR);
  // for(int i = 0; i < 4; ++i){
  //   printf("p2DPoints[%d] = %d , %d\n", i , p2DPoints[i].x, p2DPoints[i].y);
  // }
  g_dFrames++;

  printf("Z2#: %d\n", g_depth.cols * g_depth.rows * g_depth.channels());
  //printf("C3: %d\n", sizeof(*g_color.data));

  memcpy(g_depth.data, data.depthMap, data.depthMap.size()*2);
  //cv::imshow("depth", g_depth);
  //cv::waitKey(1);

  // Quit the main loop after 200 depth frames received
  //if (g_dFrames == 200)
  //    g_context.quit();
}

/*----------------------------------------------------------------------------*/
// void configureAudioNode()
// {
//     g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

//     AudioNode::Configuration config = g_anode.getConfiguration();
//     config.sampleRate = 44100;

//     try 
//     {
//         g_context.requestControl(g_anode,0);

//         g_anode.setConfiguration(config);
        
//         g_anode.setInputMixerLevel(0.5f);
//     }
//     catch (ArgumentException& e)
//     {
//         printf("Argument Exception: %s\n",e.what());
//     }
//     catch (UnauthorizedAccessException& e)
//     {
//         printf("Unauthorized Access Exception: %s\n",e.what());
//     }
//     catch (ConfigurationException& e)
//     {
//         printf("Configuration Exception: %s\n",e.what());
//     }
//     catch (StreamingException& e)
//     {
//         printf("Streaming Exception: %s\n",e.what());
//     }
//     catch (TimeoutException&)
//     {
//         printf("TimeoutException\n");
//     }
// }

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
  g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

  DepthNode::Configuration config = g_dnode.getConfiguration();
  config.frameFormat = FRAME_FORMAT_QVGA;
  config.framerate = 30;
  config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
  config.saturation = true;

  g_dnode.setEnableVertices(true);
  g_dnode.setEnableDepthMap(true);

  try 
    {
      g_context.requestControl(g_dnode,0);

      g_dnode.setConfiguration(config);
    }
  catch (ArgumentException& e)
    {
      printf("Argument Exception: %s\n",e.what());
    }
  catch (UnauthorizedAccessException& e)
    {
      printf("Unauthorized Access Exception: %s\n",e.what());
    }
  catch (IOException& e)
    {
      printf("IO Exception: %s\n",e.what());
    }
  catch (InvalidOperationException& e)
    {
      printf("Invalid Operation Exception: %s\n",e.what());
    }
  catch (ConfigurationException& e)
    {
      printf("Configuration Exception: %s\n",e.what());
    }
  catch (StreamingException& e)
    {
      printf("Streaming Exception: %s\n",e.what());
    }
  catch (TimeoutException&)
    {
      printf("TimeoutException\n");
    }

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
  // connect new color sample handler
  g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

  ColorNode::Configuration config = g_cnode.getConfiguration();
  //config.frameFormat = FRAME_FORMAT_WXGA_H;
  config.frameFormat = FRAME_FORMAT_VGA;
  config.compression = COMPRESSION_TYPE_MJPEG;
  config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
  config.framerate = 30;

  g_cnode.setEnableColorMap(true);

  try 
    {
      g_context.requestControl(g_cnode,0);

      g_cnode.setConfiguration(config);
    }
  catch (ArgumentException& e)
    {
      printf("Argument Exception: %s\n",e.what());
    }
  catch (UnauthorizedAccessException& e)
    {
      printf("Unauthorized Access Exception: %s\n",e.what());
    }
  catch (IOException& e)
    {
      printf("IO Exception: %s\n",e.what());
    }
  catch (InvalidOperationException& e)
    {
      printf("Invalid Operation Exception: %s\n",e.what());
    }
  catch (ConfigurationException& e)
    {
      printf("Configuration Exception: %s\n",e.what());
    }
  catch (StreamingException& e)
    {
      printf("Streaming Exception: %s\n",e.what());
    }
  catch (TimeoutException&)
    {
      printf("TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
  if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
      g_dnode = node.as<DepthNode>();
      configureDepthNode();
      g_context.registerNode(node);
    }

  if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
      g_cnode = node.as<ColorNode>();
      configureColorNode();
      g_context.registerNode(node);
    }

  cv::namedWindow("color");
  cv::namedWindow("depth");

  cv::namedWindow("vote");

  //  cv::namedWindow("test");

  // if ((node.is<AudioNode>())&&(!g_anode.isSet()))
  // {
  //     g_anode = node.as<AudioNode>();
  //     configureAudioNode();
  //     g_context.registerNode(node);
  // }
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
  configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
  // if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
  //     g_anode.unset();
  if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
    g_cnode.unset();
  if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
    g_dnode.unset();
  printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
  if (!g_bDeviceFound)
    {
      data.device.nodeAddedEvent().connect(&onNodeConnected);
      data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
      g_bDeviceFound = true;
    }

  cv::destroyAllWindows();
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
  g_bDeviceFound = false;
  printf("Device disconnected\n");
}

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
  CConfig		conf;	 // setting
  std::vector<CDataset> dataSet; // training data name list and grand truth

  //read argument
  //check argument
  if(argc < 2) {
    cout << "Usage: ./learning [config.xml]"<< endl;
    conf.loadConfig("config.xml");
  } else
    conf.loadConfig(argv[1]);

  if(argc < 3)
    conf.off_tree = 0;
  else
    conf.off_tree = atoi(argv[2]);

  conf.demoMode = 1;
  // create random forest class
  //CRForest forest(conf);
  g_forest = NULL;
  g_forest = new CRForest(conf);

  g_forest->loadForest();

  g_calib = new CCalibDS325;

  g_calib->loadParameters("intrinsics.yml", "extrinsics.yml");

  //create tree direct
  //string opath(conf.outpath);
  //std::cout << "kokomade kitayo" << std::endl;
  //std::cout << conf.outpath << std::endl;
  //opath.erase(opath.find_last_of(PATH_SEP));
  //string execstr = "mkdir ";
  //execstr += opath;
  //system( execstr.c_str() );

  g_context = Context::create("localhost");

  g_context.deviceAddedEvent().connect(&onDeviceConnected);
  g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

  // Get the list of currently connected devices
  vector<Device> da = g_context.getDevices();

  // init image
  //g_color = cv::Mat(720, 1280, CV_8UC3);
  g_color = cv::Mat(480, 640, CV_8UC3);
  g_depth = cv::Mat(240, 320, CV_16UC1);

  // We are only interested in the first device
  if (da.size() >= 1)
    {
      g_bDeviceFound = true;

      da[0].nodeAddedEvent().connect(&onNodeConnected);
      da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

      vector<Node> na = da[0].getNodes();
        
      printf("Found %u nodes\n",(uint)na.size());
        
      for (int n = 0; n < (int)na.size();n++)
	configureNode(na[n]);
    }

  

  g_context.startNodes();

  g_context.run();

  g_context.stopNodes();

  if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
  if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
  //if (g_anode.isSet()) g_context.unregisterNode(g_anode);

  printf("close nodes");

  if (g_pProjHelper)
    delete g_pProjHelper;

  if (g_forest)
    delete g_forest;

  if (g_calib)
    delete g_calib;

  return 0;
}
