#include <cnn_pose_estimator/validation_registration.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>
#include <time.h>

using validation_registration::ValidationRegistration;

ValidationRegistration::ValidationRegistration(ros::NodeHandle nh, ros::NodeHandle n)
  :nh_(nh)
  ,model_filename_(n.param<std::string>("model_filename", "hv6"))
  ,algorithm_name_(n.param<std::string>("method_name", "ICP"))
  ,sensor_frame_id_(n.param<std::string>("sensor_frame_id", "photoneo_center_optical_frame"))
  ,registrated_frame_id_(n.param<std::string>("registrated_frame_id", "jig_HV6_0"))
  ,leaf_(n.param<float>("leaf", 0))
  ,model_plane_threshold_(n.param<float>("model_plane_threshold", 0.005))  // HV6
  // ,plane_threshold_(0.016) // HV8
  ,scene_plane_threshold_(n.param<float>("scene_plane_threshold", 0.003))  // HV6
  // ,scene_plane_threshold_(0.012); // HV8
  ,model_planable_ratio_(n.param<float>("model_planable_ratio", 0.1))
  ,scene_planable_ratio_(n.param<float>("scene_planable_ratio", 0.3))
  ,cloud_flag_(false)
  ,timestamp_(0)

  ,x_(0)
  ,y_(0)
  ,z_(0)
  ,roll_(0)
  ,pitch_(0)
  ,yaw_(0)
  ,x_min_(0)
  ,y_min_(0)
  ,z_min_(0)
  ,roll_min_(0)
  ,pitch_min_(0)
  ,yaw_min_(0)
  ,x_max_(0)
  ,y_max_(0)
  ,z_max_(0)
  ,roll_max_(0)
  ,pitch_max_(0)
  ,yaw_max_(0)
  ,validation_flag_(false)

  ,registrate_transform_()
  ,origin_model_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>)
  ,moved_model_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>)
  ,downdownsampled_model_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>)
  ,downdownsampled_scene_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>)
  ,scene_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>)
  ,converted_model_(new pcl::PointCloud<pcl::PointXYZRGBA>)
  ,converted_scene_(new pcl::PointCloud<pcl::PointXYZRGBA>)
  ,registrated_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>)
{
  transformed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/model_pc", 1);
  registrated_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/registrated_model_pc", 1);
  tf_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/end/registrated_tf", 10);
  object_cloud_sub_ = nh_.subscribe(n.param<std::string>("source_pc", "/filtered_pointcloud"), 1, &ValidationRegistration::getSceneCloud, this);

  ValidationRegistration::loadModelCloud(origin_model_cloud_);
}

void ValidationRegistration::getSceneCloud(const sensor_msgs::PointCloud2::ConstPtr& scene)
{
  timestamp_ = scene->header.stamp;
  std::cout << "timestamp : " << timestamp_ << std::endl;
  // convert sensor cloud
  pcl::fromROSMsg(*scene, *scene_cloud_);

  std::cout << "scene_cloud : " << scene_cloud_->size() << std::endl;
  cloud_flag_ = true;
}

bool ValidationRegistration::surfaceRemove(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double threshold, double ratio)
{
  // 点群が0でないか確認
  if (cloud->points.size() == 0)
  {
    std::cout << "no cloud data" << std::endl;
    return true;
  }

  //　平面検出（pclのチュートリアル通り）
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  seg.setOptimizeCoefficients(true);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  seg.setInputCloud(cloud);
  seg.setModelType(pcl::SACMODEL_PLANE);  //モデル
  seg.setMethodType(pcl::SAC_RANSAC);     //検出手法
  seg.setMaxIterations(200);
  seg.setDistanceThreshold(threshold);  //閾値
  seg.segment(*inliers, *coefficients);

  // 平面除去を終わらせるかどうか：検出した平面が，前の平面除去した状態の点群のfinishRemovePlaneRatioで指定された割合未満の点群サイズであれば，それは平面でないとみなして点群除去処理を終了（finishRemovePlaneRatioは0.0~1.0:環境に合わせてください）
  if (inliers->indices.size() < cloud->points.size() * ratio)
  {
    return true;
  }

  // 平面除去
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true にすると平面を除去、false にすると平面以外を除去
  extract.filter(*cloud);
  std::cout << "remove plane" << std::endl;

  return false;
}

void ValidationRegistration::downsampleCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud, float leaf)
{
  // scene downsampling
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(leaf, leaf, leaf);
  sor.filter(*output_cloud);
}

void ValidationRegistration::loadModelCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
  std::cout << "algorithm：" << algorithm_name_ << std::endl;
  std::string model_filepath = ros::package::getPath("cnn_pose_estimator") + "/pcds/";
  std::string filepath = model_filepath + model_filename_ + ".pcd";
  pcl::io::loadPCDFile(filepath, *cloud);

  // 点群の中からnanを消す
  std::vector<int> dummy;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, dummy);

  // // 平面除去
  // while (true)
  // {
  //   bool model_flag = surfaceRemove(cloud, model_plane_threshold_, model_planable_ratio_);
  //   model_flag = true;
  //   if (model_flag == true)
  //   {
  //     break;
  //   }
  // }

  // model downsampling
  downsampleCloud(cloud, cloud, leaf_);
  std::cout << "model leaf size : " << leaf_ << std::endl;
  std::cout << "downsampled_model_cloud : " << cloud->size() << std::endl;
  std::cout << std::endl;
}

void ValidationRegistration::transformCloudFrame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud, std::string from_frame, std::string to_frame)
{
  sensor_msgs::PointCloud input_cloud_msg;
  sensor_msgs::PointCloud2 input_cloud_msg2;
  sensor_msgs::PointCloud transformed_cloud_msg;
  sensor_msgs::PointCloud2 transformed_cloud_msg2;
  pcl::toROSMsg(*input_cloud, input_cloud_msg2);
  sensor_msgs::convertPointCloud2ToPointCloud(input_cloud_msg2, input_cloud_msg);
  input_cloud_msg.header.frame_id = from_frame;

  tf::StampedTransform transform;
  tf_listener_.waitForTransform(from_frame, to_frame, ros::Time(0), ros::Duration(100.0));
  tf_listener_.transformPointCloud(to_frame, ros::Time(0), input_cloud_msg, from_frame, transformed_cloud_msg);
  sensor_msgs::convertPointCloudToPointCloud2(transformed_cloud_msg, transformed_cloud_msg2);
  // tf_listener_.lookupTransform(from_frame, to_frame, ros::Time(0), transform);
  // Eigen::Matrix4f mat;
  // pcl_ros::transformAsMatrix(transform, mat);
  // pcl_ros::transformPointCloud(input_cloud_msg, transformed_cloud_msg, transform);

  // while(ros::ok())
  // {
  //   if(pcl_ros::transformPointCloud(mat, input_cloud_msg, transformed_cloud_msg))
  //   // if (pcl_ros::transformPointCloud(to_frame, input_cloud_msg, transformed_cloud_msg, tf_listener_))
  //   {
  //     break;
  //   }
  //   else
  //   {
  //     std::cout << "Could NOT find target tf" << std::endl;
  //     ros::Duration(1.0).sleep();
  //   }
  // }
  pcl::fromROSMsg(transformed_cloud_msg2, *transformed_cloud);
}

void ValidationRegistration::registrateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud, std::string method)
{
  if (method == "ICP")
  {
    // ICP algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setInputSource(input_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaximumIterations(5000);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-10);

    //変換matrixを表示する
    icp.align(*registrated_cloud_);
    registrate_transform_ = (icp.getFinalTransformation()).cast<double>();
    // Eigen::Matrix4d registrate_transform_inverse = Eigen::Matrix4d::Identity();
    // registrate_transform_inverse = (icp.getFinalTransformation()).cast<double>();
    // registrate_transform_ = registrate_transform_inverse.inverse();

    std::cout << "Transformation matrix:" << std::endl;
    std::cout << registrate_transform_ << std::endl;
    ROS_INFO_STREAM("score : " << icp.getFitnessScore());
  }
  else if (method == "NDT")
  {
    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZRGBA, pcl::PointXYZRGBA> ndt;

    // Setting scale dependent NDT parameters
    ndt.setTransformationEpsilon(1e-3);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.01);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(0.01);
    // ndt.setMaximumIterations (35);

    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(target_cloud);

    // Calculating required rigid transform to align the input cloud to the
    // target cloud.
    ndt.align(*registrated_cloud_);
    Eigen::Matrix4d registrate_transform_inverse = Eigen::Matrix4d::Identity();
    registrate_transform_inverse = (ndt.getFinalTransformation()).cast<double>();
    registrate_transform_ = registrate_transform_inverse.inverse();
  }
}

void ValidationRegistration::saveData(double xe, double ye, double ze, double roe, double pie, double yae)
{
  char newfile[150] = "/home/hondasora/denso_project/src/denso_pkgs/denso_recognition/cnn_pose_estimator/data/validation_registration.csv";
  FILE *fp;
  fp = fopen(newfile, "a+");
  std::cout << "filepath : " << newfile << std::endl;
  std::cout << "--------------return file-------------- : " << fp << std::endl;

  fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf\n",
       x_, y_, z_, roll_, pitch_, yaw_, 1111, xe, ye, ze, roe, pie, yae);
  fclose(fp);

  std::cout << "save data       : " << x_ << "," << y_ << "," << z_ << "," << roll_ << "," << pitch_ << "," << yaw_ << std::endl;
  std::cout << "registrate error: " << xe << "," << ye << "," << ze << "," << roe << "," << pie << "," << yae << std::endl;
}

void ValidationRegistration::checkError()
{
  ROS_INFO("Conversion Start!!");
  Eigen::Affine3d registrate_transform_affine(registrate_transform_);
  Eigen::Vector3d trans = registrate_transform_affine.translation(); // 並進
  Eigen::Matrix3d rot = registrate_transform_affine.rotation(); // 回転
  Eigen::Vector3d euler = rot.eulerAngles(0,1,2); // オイラー角．

  if ((fabs(euler[0]) > (M_PI/2)) && (fabs(euler[1])>(M_PI/2)) && (fabs(euler[2])>(M_PI/2)))
  {
    if (euler[0] > 0)
    {
      euler[0] = euler[0] - M_PI;
    }
    else
    {
      euler[0] = euler[0] + M_PI;
    }

    if (euler[1] > 0)
    {
      euler[1] = euler[1] - M_PI;
    }
    else
    {
      euler[1] = euler[1] + M_PI;
    }

    if (euler[2] > 0)
    {
      euler[2] = euler[2] - M_PI;
    }
    else
    {
      euler[2] = euler[2] + M_PI;
    }
  }

  double x_error = trans[0] - x_;
  double y_error = trans[1] - y_;
  double z_error = trans[2] - z_;
  double roll_error = euler[0] - roll_;
  double pitch_error = euler[1] - pitch_;
  double yaw_error = euler[2] - yaw_;
  std::cout << "actual value" << std::endl;
  std::cout << " X    : " << x_ << std::endl;
  std::cout << " Y    : " << y_ << std::endl;
  std::cout << " Z    : " << z_ << std::endl;
  std::cout << " ROLL : " << roll_ << std::endl;
  std::cout << " PITCH: " << pitch_ << std::endl;
  std::cout << " YAW  : " << yaw_ << std::endl;

  std::cout << "estimated value" << std::endl;
  std::cout << " X    : " << trans[0] << std::endl;
  std::cout << " Y    : " << trans[1] << std::endl;
  std::cout << " Z    : " << trans[2] << std::endl;
  std::cout << " ROLL : " << euler[0] << std::endl;
  std::cout << " PITCH: " << euler[1] << std::endl;
  std::cout << " YAW  : " << euler[2] << std::endl;

  std::cout << "estimated error" << std::endl;
  std::cout << " X    : " << x_error << std::endl;
  std::cout << " Y    : " << y_error << std::endl;
  std::cout << " Z    : " << z_error << std::endl;
  std::cout << " ROLL : " << roll_error << std::endl;
  std::cout << " PITCH: " << pitch_error << std::endl;
  std::cout << " YAW  : " << yaw_error << std::endl;

  double x_threthold = 0.002;
  double y_threthold = 0.002;
  double z_threthold = 0.002;
  double roll_threthold = 5*M_PI/180;
  double pitch_threthold = 5*M_PI/180;
  double yaw_threthold = 5*M_PI/180;

  if((fabs(x_error) > x_threthold)||(fabs(y_error) > y_threthold)||(fabs(z_error) > z_threthold)||(fabs(roll_error) > roll_threthold)||(fabs(pitch_error) > pitch_threthold)||(fabs(yaw_error) > yaw_threthold)||
      (fabs(x_) > 0.16)||(fabs(y_) > 0.16)||(fabs(z_) > 0.16)||(fabs(roll_) > M_PI)||(fabs(pitch_) > M_PI)||(fabs(yaw_) > M_PI))
  {
    saveData(x_error, y_error, z_error, roll_error, pitch_error, yaw_error);
    if(fabs(x_) > 0)
    {
      if(fabs(x_min_)==0 && fabs(x_max_)==0)
      {
        x_min_ = x_;
        x_max_ = x_;
      }
      else
      {
        if (fabs(x_) > fabs(x_max_))
        {
          x_max_ = x_;
        }
        else
        {
          x_min_ = x_;
        }
      }
    }
    else if(fabs(y_) > 0)
    {
      if(fabs(y_min_)==0 && fabs(y_max_)==0)
      {
        y_min_ = y_;
        y_max_ = y_;
      }
      else
      {
        if (fabs(y_) > fabs(y_max_))
        {
          y_max_ = y_;
        }
        else
        {
          y_min_ = y_;
        }
      }
    }
    else if(fabs(z_) > 0)
    {
      if(fabs(z_min_)==0 && fabs(z_max_)==0)
      {
        z_min_ = z_;
        z_max_ = z_;
      }
      else
      {
        if (fabs(z_) > fabs(z_max_))
        {
          z_max_ = z_;
        }
        else
        {
          z_min_ = z_;
        }
      }
    }
    else if(fabs(roll_) > 0)
    {
      if(fabs(roll_min_)==0 && fabs(roll_max_)==0)
      {
        roll_min_ = roll_;
        roll_max_ = roll_;
      }
      else
      {
        if (fabs(roll_) > fabs(roll_max_))
        {
          roll_max_ = roll_;
        }
        else
        {
          roll_min_ = roll_;
        }
      }
    }
    else if(fabs(pitch_) > 0)
    {
      if(fabs(pitch_min_)==0 && fabs(pitch_max_)==0)
      {
        pitch_min_ = pitch_;
        pitch_max_ = pitch_;
      }
      else
      {
        if (fabs(pitch_) > fabs(pitch_max_))
        {
          pitch_max_ = pitch_;
        }
        else
        {
          pitch_min_ = pitch_;
        }
      }
    }
    else if(fabs(yaw_) > 0)
    {
      if(fabs(yaw_min_)==0 && fabs(yaw_max_)==0)
      {
        yaw_min_ = yaw_;
        yaw_max_ = yaw_;
      }
      else
      {
        if (fabs(yaw_) > fabs(yaw_max_))
        {
          yaw_max_ = yaw_;
        }
        else
        {
          yaw_min_ = yaw_;
        }
      }
    }

    // reset tlanslation value
    x_ = 0;
    y_ = 0;
    z_ = 0;
    roll_ = 0;
    pitch_ = 0;
    yaw_ = 0;

    validation_flag_ = true;
  }


}

void ValidationRegistration::moveModelCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud)
{
    Eigen::Translation<double, 3> trans(x_, y_, z_);
    Eigen::Affine3d rot = Eigen::Affine3d(Eigen::AngleAxisd(roll_, Eigen::Vector3d::UnitX()))
                        * Eigen::Affine3d(Eigen::AngleAxisd(pitch_, Eigen::Vector3d::UnitY()))
                        * Eigen::Affine3d(Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d affine = trans * rot;
    tf::Transform transform_tf;
    tf::transformEigenToTF(affine, transform_tf);
    const tf::Transform transform = transform_tf;
    Eigen::Matrix4f mat;
    pcl_ros::transformAsMatrix(transform_tf, mat);

    // br_.sendTransform(tf::StampedTransform(transform_tf,
    // ros::Time::now(), "object_origin", "converted_object_origin"));
    // pcl_ros::transformPointCloud(input_cloud, transformed_cloud, transform);

    // geometry_msgs::Transform transform_msg;
    // tf::transformTFToMsg(transform_tf,transform_msg);
    // geometry_msgs::TransformStamped transform_msg_stamped;
    // transform_msg_stamped.transform = transform_msg;
    // transform_msg_stamped.header.frame_id = "object_origin";
    // transform_msg_stamped.child_frame_id = "converted_object_origin";
    // std::cout << "child_frame_id : " << transform_msg_stamped.child_frame_id << std::endl;;
    // tf_pub_.publish(transform_msg_stamped);

    sensor_msgs::PointCloud2 input_cloud_msg;
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(*input_cloud, input_cloud_msg);
    pcl_ros::transformPointCloud(mat, input_cloud_msg, transformed_cloud_msg);
    pcl::fromROSMsg(transformed_cloud_msg, *transformed_cloud);

    transformed_cloud_msg.header.stamp = ros::Time(0);
    transformed_cloud_msg.header.frame_id = "object_origin";
    registrated_cloud_pub_.publish(transformed_cloud_msg);

}

void ValidationRegistration::publish()
{
  if(cloud_flag_)
  {
    clock_t start = clock();


    // downsample scene cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampled_scene_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    downsampleCloud(scene_cloud_, downsampled_scene_cloud, leaf_);
    std::cout << "scene leaf size : " << leaf_ << std::endl;
    std::cout << "downsampled_scene_cloud : " << downsampled_scene_cloud->size() << std::endl;

    // 位置ずれ
    while (validation_flag_ == false)
    {
      x_ += 0.004;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;
    while (validation_flag_ == false)
    {
      x_ -= 0.004;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_ == false)
    {
      y_ += 0.004;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;
    while (validation_flag_ == false)
    {
      y_ -= 0.004;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_ == false)
    {
      z_ += 0.004;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;
    while (validation_flag_ == false)
    {
      z_ -= 0.004;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    //　回転ずれ
    while (validation_flag_ == false)
    {
      roll_ += M_PI/90;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;
    while (validation_flag_ == false)
    {
      roll_ -= M_PI/90;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_ == false)
    {
      pitch_ += M_PI/90;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;
    while (validation_flag_ == false)
    {
      pitch_ -= M_PI/90;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_ == false)
    {
      yaw_ += M_PI/90;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;
    while (validation_flag_ == false)
    {
      yaw_ -= M_PI/90;
      // object_origin から offset を加えたframeを作成
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    double resol_x_min = x_min_ / 20;
    double resol_y_min = y_min_ / 20;
    double resol_z_min = z_min_ / 20;
    double resol_roll_min = roll_min_ / 20;
    double resol_pitch_min = pitch_min_ / 20;
    double resol_yaw_min = yaw_min_ / 20;

    double resol_x_max = x_max_ / 20;
    double resol_y_max = y_max_ / 20;
    double resol_z_max = z_max_ / 20;
    double resol_roll_max = roll_max_ / 20;
    double resol_pitch_max = pitch_max_ / 20;
    double resol_yaw_max = yaw_max_ / 20;

    std::cout << "resolution valdiation" << std::endl;
    std::cout << "min x     : " << x_min_ << std::endl;
    std::cout << "min y     : " << y_min_ << std::endl;
    std::cout << "min z     : " << z_min_ << std::endl;
    std::cout << "min roll  : " << roll_min_ << std::endl;
    std::cout << "min pitch : " << pitch_min_ << std::endl;
    std::cout << "min yaw   : " << yaw_min_ << std::endl;

    std::cout << "max x     : " << x_max_ << std::endl;
    std::cout << "max y     : " << y_max_ << std::endl;
    std::cout << "max z     : " << z_max_ << std::endl;
    std::cout << "max roll  : " << roll_max_ << std::endl;
    std::cout << "max pitch : " << pitch_max_ << std::endl;
    std::cout << "max yaw   : " << yaw_max_ << std::endl;

    // 位置ずれの組み合わせ
    while (validation_flag_ == false)
    {
      x_ += resol_x_min;
      y_ += resol_y_min;
      z_ += resol_z_min;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_ == false)
    {
      x_ += resol_x_max;
      y_ += resol_y_max;
      z_ += resol_z_max;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    // 回転ずれの組み合わせ
    while (validation_flag_ == false)
    {
      roll_ += resol_roll_min;
      pitch_ += resol_pitch_min;
      yaw_ += resol_yaw_min;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_ == false)
    {
      roll_ += resol_roll_max;
      pitch_ += resol_pitch_max;
      yaw_ += resol_yaw_max;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    // 位置ずれ＋回転ずれの組み合わせ
    while (validation_flag_==false)
    {
      x_ += resol_x_min;
      y_ += resol_y_min;
      z_ += resol_z_min;
      roll_ += resol_roll_min;
      pitch_ += resol_pitch_min;
      yaw_ += resol_yaw_min;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_==false)
    {
      x_ += resol_x_max;
      y_ += resol_y_max;
      z_ += resol_z_max;
      roll_ += resol_roll_max;
      pitch_ += resol_pitch_max;
      yaw_ += resol_yaw_max;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    // ダウンサンプリング
    downsampleCloud(downsampled_scene_cloud, downdownsampled_scene_cloud_, leaf_*2);
    // ダウンサンプリング（scene cloud）
    while (validation_flag_==false)
    {
      x_ += resol_x_min;
      y_ += resol_y_min;
      z_ += resol_z_min;
      roll_ += resol_roll_min;
      pitch_ += resol_pitch_min;
      yaw_ += resol_yaw_min;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downdownsampled_scene_cloud_, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_==false)
    {
      x_ += resol_x_max;
      y_ += resol_y_max;
      z_ += resol_z_max;
      roll_ += resol_roll_max;
      pitch_ += resol_pitch_max;
      yaw_ += resol_yaw_max;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downdownsampled_scene_cloud_, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, moved_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    // ダウンサンプリング（model cloud）
    downsampleCloud(moved_model_cloud_, downdownsampled_model_cloud_, leaf_*10);
    while (validation_flag_==false)
    {
      x_ += resol_x_min;
      y_ += resol_y_min;
      z_ += resol_z_min;
      roll_ += resol_roll_min;
      pitch_ += resol_pitch_min;
      yaw_ += resol_yaw_min;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, downdownsampled_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    while (validation_flag_==false)
    {
      x_ += resol_x_max;
      y_ += resol_y_max;
      z_ += resol_z_max;
      roll_ += resol_roll_max;
      pitch_ += resol_pitch_max;
      yaw_ += resol_yaw_max;
      moveModelCloud(origin_model_cloud_, moved_model_cloud_);
      // transform cloud from sensor frame to world
      transformCloudFrame(downsampled_scene_cloud, converted_scene_, "/world", "object_origin");
      // 精密位置合わせを行う
      registrateCloud(converted_scene_, downdownsampled_model_cloud_, algorithm_name_);
      // ICPの誤差を計算
      checkError();
    }
    validation_flag_ = false;
    std::cout << __LINE__ << std::endl;

    clock_t end = clock();
    const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
    printf("time %lf[ms]\n\n", time);

    char newfile[150] = "/home/hondasora/denso_project/src/denso_pkgs/denso_recognition/cnn_pose_estimator/data/validation_registration.csv";
    FILE *fp;
    fp = fopen(newfile, "a+");
    std::cout << "filepath : " << newfile << std::endl;
    std::cout << "--------------return file-------------- : " << fp << std::endl;

    fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf\n",
         resol_x_min*20, resol_y_min*20, resol_z_min*20, resol_roll_min*20, resol_pitch_min*20, resol_yaw_min*20);
    fprintf(fp, "%lf,%lf,%lf,%lf,%lf,%lf\n",
         resol_x_max*20, resol_y_max*20, resol_z_max*20, resol_roll_max*20, resol_pitch_max*20, resol_yaw_max*20);
    fclose(fp);
    ros::shutdown();
  }
}
