
#include <iostream>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
  for(auto itr=kptMatches.begin();itr!=kptMatches.end();itr++)    
  {
    auto currpts = kptsCurr.at(itr->trainIdx).pt;
    if((boundingBox.roi).contains(currpts))
    {
      boundingBox.kptMatches.push_back(*itr);
    }
  }
  double sum = 0.0;
  for(auto it = boundingBox.kptMatches.begin(); it!= boundingBox.kptMatches.end(); ++it)
  {
    cv::KeyPoint currkpts = kptsCurr.at(it->trainIdx);
    cv::KeyPoint prevkpts = kptsPrev.at(it->queryIdx);
    double distance = cv::norm(currkpts.pt - prevkpts.pt);
    sum += distance;
  }
  
  double mean = sum / boundingBox.kptMatches.size();
  const double const_ratio= 1.5;
  
  for(auto it = boundingBox.kptMatches.begin(); it!= boundingBox.kptMatches.end();)
  {
    cv::KeyPoint currkpts = kptsCurr.at(it->trainIdx);
    cv::KeyPoint prevkpts = kptsPrev.at(it->queryIdx);
    double distance = cv::norm(currkpts.pt - prevkpts.pt);
  
    if(distance > (const_ratio * mean))
    {
      boundingBox.kptMatches.erase(it);
    }
    else
    {
      ++it;
    }
  }
}
   
   
  
// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC)
{

  //cout << " HELLO I AM CALCULATING TTC WITH CAMERA" << endl;
  vector<double> dist_ratios;
  for(auto it_1 = kptMatches.begin(); it_1 != kptMatches.end() - 1; ++it_1)
  {
       //cout << "kptMatches SIZE = " << kptMatches.size() << endl;
      //cout << " TTC WITH CAMERA IN PROCESS..." << endl;
      
      cv::KeyPoint kptcurrs_1 = kptsCurr.at(it_1->trainIdx); //one keypoint in curr     
      cv::KeyPoint kptprevs_1 = kptsPrev.at(it_1->queryIdx); //and its matched in prev
 
      
       //cout << " Moving to the inner loop..." << endl; 
      for(auto it_2 = kptMatches.begin() + 1; it_2 !=kptMatches.end(); ++it_2)
      {
        //cout << " TTC WITH CAMERA IN PROCESS ##2..." << endl;  
        cv::KeyPoint kptcurrs_2 =kptsCurr.at(it_2->trainIdx); 
        cv::KeyPoint kptprevs_2 =kptsPrev.at(it_2->queryIdx); 
        
        double min_dist = 100;
        //double curr_dist =10.5;
        //double prev_dist = 9.6;
        double curr_dist = cv::norm(kptcurrs_2.pt - kptcurrs_1.pt);
        double prev_dist = cv::norm(kptprevs_2.pt - kptprevs_1.pt);
        if(prev_dist >std::numeric_limits<double>::epsilon() && curr_dist >= min_dist)
        {
          double dist_ratio = curr_dist / prev_dist;
          dist_ratios.push_back(dist_ratio);
        }
      }
  }
  if(dist_ratios.size()==0)
  {
    TTC = NAN;
    return;
  }
   
   typedef vector<double>::size_type vec_sz;
   sort(dist_ratios.begin(), dist_ratios.end());
   vec_sz size = dist_ratios.size();
   vec_sz mid = size/2;
   double median_ratio = (size%2 ==0? (dist_ratios[mid] + dist_ratios[mid -1]) /2 : dist_ratios[mid]);
   double dT = 1/frameRate;
  TTC = -dT/(1 - median_ratio); 
   cout << " TTC (WITH Camera) = " << TTC << " seconds " << endl;
  
   //std::vector<double>ttcamera;
   //ttcamera.push_back(TTC);
 
}


std::vector<LidarPoint> LidarClustering(std::vector<LidarPoint> LidarPts, float clusterTolerance)
{
  KdTree * tree = new KdTree();
  std::vector<std::vector<float>>points;
  for(int i = 0; i<LidarPts.size(); ++i)
  {
    points.push_back(std::vector<float>({static_cast<float>(LidarPts[i].x),
                                         static_cast<float>(LidarPts[i].y),
                                         static_cast<float>(LidarPts[i].z)}));
    
    tree->insert(std::vector<float>({static_cast<float>(LidarPts[i].x),
                                         static_cast<float>(LidarPts[i].y),
                                         static_cast<float>(LidarPts[i].z)}), i);
  }
  
  std::vector<LidarPoint> clusters;
  std::vector<std::vector<int>> clusterindices = euclideanCluster(points, tree, clusterTolerance);
  for(auto clusterindex: clusterindices)
  {
    std::vector<LidarPoint>lidarclusters;
    for(auto index: clusterindex)
    {
      lidarclusters.push_back(LidarPts[index]);
    }
    if(lidarclusters.size() > clusters.size())
    {
      clusters = lidarclusters;
    }
  }
  return clusters;
}


void clusterhelper(int index, const std::vector< std::vector<float> >&points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree *tree, float distanceTol)  
{
  processed[index] = true;
  cluster.push_back(index);
  std::vector<int> nearby = tree->search(points[index], distanceTol);
  for(int id: nearby)
  {
    if(!processed[id])
    {
      clusterhelper(id, points, cluster, processed, tree, distanceTol);
    }
  }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
  // TODO: Fill out this function to return list of indices for each cluster
  std::vector<std::vector<int>> clusters;
  std::vector<bool> processed(points.size(), false);
  
  int i =0;
  while(i < points.size())
  {
    if(processed[i])
    {
      i++;
      continue;
    }
    std::vector<int> cluster;
    clusterhelper(i, points, cluster, processed, tree, distanceTol);
    clusters.push_back(cluster);
    i++;
  }
  return clusters;

}



void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
  float clusterTolerance = 0.1;
  vector<LidarPoint>LidarPointsPrevCluster = LidarClustering(lidarPointsPrev, clusterTolerance);
  
  vector<LidarPoint>LidarPointsCurrCluster = LidarClustering(lidarPointsCurr, clusterTolerance);
  
  double minXPrev = 1e9, minXCurr = 1e9;
 
  for(auto it:LidarPointsPrevCluster)
  {
    minXPrev = minXPrev>it.x ? it.x : minXPrev;
  }

  for(auto it:LidarPointsCurrCluster)
  {
    minXCurr = minXCurr>it.x ? it.x : minXCurr;
  }
  double dT = 1/frameRate; 
 
  TTC = minXCurr * dT / (minXPrev-minXCurr);
  cout << " TTC (WITH LIDAR) = " << TTC << " seconds " << endl;
 
 }


template<typename KeyType, typename ValueType>
std::pair<KeyType, ValueType> FindMax(const std::map<KeyType, ValueType>& x) {
    using pairtype = std::pair<KeyType, ValueType>;
    return *std::max_element(x.begin(), x.end(), [] (const pairtype & p1, const pairtype & p2) {
        return p1.second < p2.second;
    });
}



void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
  for(std::vector<BoundingBox>::iterator previter = prevFrame.boundingBoxes.begin(); previter!=prevFrame.boundingBoxes.end(); ++previter)
  {
    map<int, int>matchboundingbox;
    for(std::vector<BoundingBox>::iterator curriter = currFrame.boundingBoxes.begin(); curriter!=currFrame.boundingBoxes.end(); ++curriter)
    { 
      for(auto it1 =matches.begin(); it1!=matches.end(); ++it1)
      {   
        auto kptsprev = prevFrame.keypoints.at(it1->queryIdx).pt;
        
        if((previter->roi).contains(kptsprev))
        {
          auto kptscurr = currFrame.keypoints.at(it1->trainIdx).pt;
          //cout << " previter BOX = " << previter->boxID << endl; 
          
          if((curriter->roi).contains(kptscurr))
          {
            //cout << " curriter BOX = " << curriter->boxID << endl;
            if(matchboundingbox.count(curriter->boxID) == 0)
            {
              matchboundingbox[curriter->boxID] =1;
            }
            else
            {
              ++matchboundingbox[curriter->boxID];
            }
          }
        }
      }
    }
    auto max=FindMax(matchboundingbox);
    bbBestMatches[previter->boxID] =max.first;
  }
}