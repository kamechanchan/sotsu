pcl::PointCloud <pcl::PointXYZ>::Ptr save_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointXYZ foreground_point;
pcl::MinCutSegmentation<pcl::PointXYZ> seg;
pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());

void executeMinCutSeg (pcl::PointCloud <pcl::PointXYZ>::Ptr input_cloud,
                       const double foreground_x, const double foreground_y, const double foreground_z,
                       const double removal_z_min, const double removal_z_max,
                       const double sigma, const double object_size, const int neighbours_num, const double src_weight)
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();

  foreground_point.x = foreground_x;
  foreground_point.y = foreground_y;
  foreground_point.z = foreground_z;
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (removal_z_min, removal_z_max);
  pass.filter (*indices);

  seg.setInputCloud (input_cloud);
  seg.setIndices (indices);

  foreground_points->points.push_back(foreground_point);
  seg.setForegroundPoints (foreground_points);

  seg.setSigma (sigma);
  seg.setRadius (object_size);
  seg.setNumberOfNeighbours (neighbours_num);
  seg.setSourceWeight (src_weight);

  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters);

  end = std::chrono::system_clock::now();

  double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
  std::cout << "TIME: " << time << "[msec]" << std::endl;

  // std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  colored_cloud = seg.getColoredCloud ();

  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt = colored_cloud->points.begin(); pt < colored_cloud->points.end(); pt++)
  {
    if (pt->g == 255)
    {
      pcl::PointXYZ buffer_point;
      buffer_point.x = pt->x;
      buffer_point.y = pt->y;
      buffer_point.z = pt->z;
      save_cloud->push_back(buffer_point);
    }
  }
  std::cout << "Extracting Point Cloud!!" << std::endl;

  pcl::io::savePCDFileASCII ("../pcd/min_cut_result.pcd", *colored_cloud);
  pcl::io::savePCDFileASCII ("../pcd/min_cut_extracted_pc.pcd", *save_cloud);
}
