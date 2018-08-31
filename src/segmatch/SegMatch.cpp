#include "segmatch/SegMatch.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <pcl/recognition/cg/geometric_consistency.h>

#include <opencv2/core/eigen.hpp>

#include "segmatch/segmenters/segmenters.hpp"


SegMatch::SegMatch(SegMatchParam segmatch_param)
{
	param = segmatch_param;

	m_rtrees = new CvRTrees;
	m_rtrees->load(param.input_rtrees_name.c_str());
	m_segmenter = create_segmenter(param);
	m_nns = NULL;
}

SegMatch::~SegMatch()
{
	delete m_rtrees;
	if(m_nns != NULL)delete m_nns;
	m_segmenter.reset();
}

void displayPerformances(unsigned int tp, unsigned int tn,
                                unsigned int fp, unsigned int fn) 
{

	cout << "TP: " << tp << ", TN: " << tn <<
		", FP: " << fp << ", FN: " << fn << "."<<endl;

	const double true_positive_rate = double(tp) / double(tp + fn);
	const double true_negative_rate = double(tn) / double(fp + tn);
	const double false_positive_rate = 1.0 - true_negative_rate;

	cout << "Accuracy (ACC): " << double(tp + tn) /
		double(tp + fp + tn + fn)<<endl;
	cout << "Sensitivity (TPR): " << true_positive_rate<<endl;
	cout << "Specificity (TNR): " << true_negative_rate<<endl;
	cout << "Precision: " << double(tp) / double(tp + fp)<<endl;
	cout << "Positive likelyhood ratio: " << true_positive_rate / false_positive_rate<<endl;
}

PointXYZ calculateCentroid(PointCloud<PointXYZI>::Ptr& cloud)
{
	int kNPoints = cloud->size();
	pcl::PointXYZ ptCenter(0.0, 0.0, 0.0);
	for (size_t i = 0u; i < kNPoints; ++i)
	{
		ptCenter.x += cloud->at(i).x;
		ptCenter.y += cloud->at(i).y;
		ptCenter.z += cloud->at(i).z;
	}
	ptCenter.x /= double(kNPoints);
	ptCenter.y /= double(kNPoints);
	ptCenter.z /= double(kNPoints);
	return ptCenter;
}

cv::Mat calculateFeature(PointCloud<PointXYZI>::Ptr& points_in)
{
	// cv::Mat feature(1,7, CV_64F);
	Mat feature(1,7, CV_32F);
	const size_t kNPoints = points_in->size();
	if (kNPoints <= 0)
	{
		cout << "kNPoints <= 0" << endl;
		getchar();
	}

	//Calculate the center of point cloud
	PointXYZ ptCenter(0.0, 0.0, 0.0);
	for (size_t i = 0u; i < kNPoints; ++i)
	{
		ptCenter.x += points_in->at(i).x;
		ptCenter.y += points_in->at(i).y;
		ptCenter.z += points_in->at(i).z;
	}
	ptCenter.x /= double(kNPoints);
	ptCenter.y /= double(kNPoints);
	ptCenter.z /= double(kNPoints);

	// Find the variances.
	PointCloud<pcl::PointXYZ> variances;
	for (size_t i = 0u; i < kNPoints; ++i)
	{
		variances.push_back(pcl::PointXYZ());
		variances[i].x = points_in->at(i).x - ptCenter.x;
		variances[i].y = points_in->at(i).y - ptCenter.y;
		variances[i].z = points_in->at(i).z - ptCenter.z;
	}

	// Find the covariance matrix. Since it is symmetric, we only bother with the upper diagonal.
	const std::vector<size_t> row_indices_to_access = {0,0,0,1,1,2};
	const std::vector<size_t> col_indices_to_access = {0,1,2,1,2,2};
	Eigen::Matrix3f covariance_matrix;
	for (size_t i = 0u; i < row_indices_to_access.size(); ++i)
	{
		const size_t row = row_indices_to_access[i];
		const size_t col = col_indices_to_access[i];
		double covariance = 0;
		for (size_t k = 0u; k < kNPoints; ++k)
		{
			covariance += variances.points[k].data[row] * variances.points[k].data[col];
		}
		covariance /= kNPoints;
		covariance_matrix(row,col) = covariance;
		covariance_matrix(col,row) = covariance;
	}

	// Compute eigenvalues of covariance matrix.
	constexpr bool compute_eigenvectors = false;
	Eigen::EigenSolver<Eigen::Matrix3f> eigenvalues_solver(covariance_matrix, compute_eigenvectors);
	std::vector<float> eigenvalues(3, 0.0);
	eigenvalues.at(0) = eigenvalues_solver.eigenvalues()[0].real();
	eigenvalues.at(1) = eigenvalues_solver.eigenvalues()[1].real();
	eigenvalues.at(2) = eigenvalues_solver.eigenvalues()[2].real();
	if (eigenvalues_solver.eigenvalues()[0].imag() != 0.0 ||
			eigenvalues_solver.eigenvalues()[1].imag() != 0.0 ||
			eigenvalues_solver.eigenvalues()[2].imag() != 0.0 )
	{
		cout << "eigenvalues_solver.eigenvalues()[0].imag() != 0.0" << endl;
		getchar();
	}

	// Sort eigenvalues from smallest to largest.
	swap_if_gt(eigenvalues.at(0), eigenvalues.at(1));
	swap_if_gt(eigenvalues.at(0), eigenvalues.at(2));
	swap_if_gt(eigenvalues.at(1), eigenvalues.at(2));

	// Normalize eigenvalues.
	double sum_eigenvalues = eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
	double e1 = eigenvalues.at(0) / sum_eigenvalues;
	double e2 = eigenvalues.at(1) / sum_eigenvalues;
	double e3 = eigenvalues.at(2) / sum_eigenvalues;
	if (e1 == e2 || e2 == e3 || e1 == e3)
	{
		cout << "e1 == e2 || e2 == e3 || e1 == e3" << endl;
		getchar();
	}

	// Store inside features.
	const double sum_of_eigenvalues = e1 + e2 + e3;
	constexpr double kOneThird = 1.0/3.0;
	if (e1 == 0.0 || sum_of_eigenvalues == 0.0)
	{
		cout << "e1 == 0.0 || sum_of_eigenvalues == 0.0" << endl;
		getchar();
	}

	const double kNormalizationPercentile = 1.0;

	const double kLinearityMax = 28890.9 * kNormalizationPercentile;    		//线性度  
	const double kPlanarityMax = 95919.2 * kNormalizationPercentile;    		//平面度
	const double kScatteringMax = 124811 * kNormalizationPercentile;    		//散射度
	const double kOmnivarianceMax = 0.278636 * kNormalizationPercentile;    	//全方差
	const double kAnisotropyMax = 124810 * kNormalizationPercentile;    		//各向异性
	const double kEigenEntropyMax = 0.956129 * kNormalizationPercentile;    	//特征熵
	const double kChangeOfCurvatureMax = 0.99702 * kNormalizationPercentile;    //曲率

	const double kNPointsMax = 13200 * kNormalizationPercentile;

	feature.at<float>(0,0) = (e1 - e2) / e1 / kLinearityMax;    
	feature.at<float>(0,1) = (e2 - e3) / e1 / kPlanarityMax;    
	feature.at<float>(0,2) = e3 / e1 / kScatteringMax;  
	feature.at<float>(0,3) = std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax;  
	feature.at<float>(0,4) = (e1 - e3) / e1 / kAnisotropyMax;   
	feature.at<float>(0,5) = (e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)) / kEigenEntropyMax;    
	feature.at<float>(0,6) = e3 / sum_of_eigenvalues / kChangeOfCurvatureMax;

	return feature;
}

cv::Mat calculateFeatureDistance(const cv::Mat& feature1, const cv::Mat& feature2)
{
	// Mat diff_out = Mat::zeros(1, 35, CV_64F);
	Mat diff_out = Mat::zeros(1, 35, CV_32F);
	Mat f_diff = cv::abs(feature1 - feature2);
	f_diff.copyTo(diff_out(Range::all(),Range(0,7)));

	Mat f1_abs = cv::abs(feature1);
	Mat f2_abs = cv::abs(feature2);

	Mat f_diff_norm_2 = f_diff/f2_abs;
	Mat f_diff_norm_1 = f_diff/f1_abs;

	f_diff_norm_2.copyTo(diff_out(Range::all(),Range(7,14)));
	f_diff_norm_1.copyTo(diff_out(Range::all(),Range(14,21)));
	// f1_abs.copyTo(diff_out(Range::all(),Range(21,28)));
	f2_abs.copyTo(diff_out(Range::all(),Range(21,28)));
	f1_abs.copyTo(diff_out(Range::all(),Range(28,35)));
	
	return diff_out;
}

void SegMatch::filterCloud(PointCloud<PointXYZI>::Ptr& cloud)
{
	PointCloud<PointXYZI>::Ptr cloud_filtered(new PointCloud<PointXYZI>);
	
	//voxelgrid filter
	VoxelGrid<PointXYZI> vg;
	vg.setInputCloud (cloud);

	vg.setLeafSize (param.voxel_grid_leaf_size, 
					param.voxel_grid_leaf_size, 
					param.voxel_grid_leaf_size);
	vg.setMinimumPointsNumberPerVoxel(param.voxel_grid_points_per_voxles);
	vg.filter (*cloud_filtered);
	(*cloud) = (*cloud_filtered);

	//outlier filter
	StatisticalOutlierRemoval<PointXYZI> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (param.outlier_removal_mean_k);
	sor.setStddevMulThresh (param.outlier_removal_dev_threshold);
	sor.filter (*cloud_filtered);
	(*cloud) = (*cloud_filtered);
}

void SegMatch::describeCloud(PointCloud<PointXYZI>::Ptr& cloud, vector<PointIndices>& clusters, vector<Segment>& segments)
{
	int id = 0;
	vector<Segment> segments_; 

	for(auto seg = clusters.begin(); seg != clusters.end(); seg++)
	{
		PointCloud<PointXYZI>::Ptr sub(new PointCloud<PointXYZI>);
		
		for(auto pnt = seg->indices.begin(); pnt != seg->indices.end(); pnt++)
		{
			sub->push_back(cloud->at(*pnt));
		}

		Segment segment;
		segment.id = id;
		segment.cloud = *sub;
		segment.centroid = calculateCentroid(sub);
		segment.feature = calculateFeature(sub);
		segments_.push_back(segment);
		id++;
	}
	segments = segments_;
}

void SegMatch::processCloud(PointCloud<PointXYZI>::Ptr& cloud, vector<Segment>& segments)
{
	PointCloud<PointXYZI>::Ptr filtered_cloud(new PointCloud<PointXYZI>);
	copyPointCloud(*cloud, *filtered_cloud);
	
	//1. filter the cloud
	// filterCloud(filtered_cloud);

	//2. segment the cloud
	vector<PointIndices> clusters;
	// segmentCloud(filtered_cloud, clusters);
	m_segmenter->segment(filtered_cloud, clusters);

	//3. calculate the feature of the cloud
	describeCloud(filtered_cloud, clusters, segments);

}


void SegMatch::setSourceCloud(PointCloud<PointXYZI>::Ptr& cloud)
{
	processCloud(cloud, m_source_segments);
}

void SegMatch::setTargetCloud(PointCloud<PointXYZI>::Ptr& cloud)
{
	processCloud(cloud, m_target_segments);

	Mat target_mat(m_target_segments.size(), 7, CV_32F);
	for(size_t i = 0; i < m_target_segments.size(); i++)
	{
		m_target_segments[i].feature.copyTo(target_mat.row(i));
	}

	cv2eigen<float>(target_mat, m_target_matrix);
	m_target_matrix.transposeInPlace();

	m_nns = NNSearchF::createKDTreeLinearHeap(m_target_matrix);
}

int SegMatch::findCandidates(vector<Match>& matches, bool one_stage)
{	
	//one_stage: true	only knn
	//one_stage: false	knn + rtrees

	vector<Match> knn_matches;
	vector<Match> rtrees_matches;

	VectorXi indices;        //result indices
	VectorXf dists2;         //features distance
	
	//1. use knn to search  [f0,f1,...fn]
	int n_knn = (param.knn_number <= m_target_segments.size()) ? param.knn_number : m_target_segments.size();
	for(size_t i = 0; i < m_source_segments.size(); i++)
	{
		MatrixXf query;
		cv2eigen<float>(m_source_segments[i].feature, query);

		query.transposeInPlace();   //column vector
		m_nns->knn(query, indices, dists2, n_knn, 0, NNSearchF::SORT_RESULTS|NNSearchF::ALLOW_SELF_MATCH);
		
		for(size_t j = 0; j < n_knn; j++)
		{
			Match match;
			match.id1 = i;				//source id
			match.id2 = indices[j];		//target id
			match.centroid1 = m_source_segments[i].centroid;
			match.centroid2 = m_target_segments[indices[j]].centroid;
			match.feature1 = m_source_segments[i].feature.clone();
			match.feature2 = m_target_segments[indices[j]].feature.clone();
			knn_matches.push_back(match);
		}
	} 

	if(one_stage)
	{
		matches = knn_matches;
		return matches.size();
	}

	//2. use rtrees to predict T([d0,d1,...dn])
	for (size_t i = 0; i < knn_matches.size(); i++)
	{
		Mat feature_diff = calculateFeatureDistance(knn_matches[i].feature1, knn_matches[i].feature2);
		Mat feature_diff_f;
		feature_diff.convertTo(feature_diff_f, CV_32F);
		float cnf = m_rtrees->predict_prob(feature_diff_f);
		if (cnf >= param.min_confidence)
		{
			knn_matches[i].confidence = cnf;
			rtrees_matches.push_back(knn_matches[i]);
		}
	}

	matches = rtrees_matches;
	return matches.size();
}



int SegMatch::filterMatches(vector<Match>& matches, 
							vector<Match>& filtered_matches, 
							Eigen::Matrix4f& transformation)
{
	transformation = Eigen::Matrix4f::Identity();
	
	if(matches.size() == 0)
	{
		cout << "No valid matches found!" << endl;
		return 0;
	}

	CorrespondencesPtr correspondences_ptr(new Correspondences());
	PointCloud<PointXYZ>::Ptr first_cloud(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr second_cloud(new PointCloud<PointXYZ>);

	for (size_t i = 0u; i < matches.size(); ++i)
	{
		// First centroid.
		PointXYZ first_centroid = matches[i].centroid1;
		first_cloud->push_back(first_centroid);
		// Second centroid.
		PointXYZ second_centroid = matches[i].centroid2;
		second_cloud->push_back(second_centroid);
		float squared_distance = 1.0 - matches[i].confidence;
		correspondences_ptr->push_back(Correspondence(i, i, squared_distance));
	}


	std::vector<Eigen::Matrix4f,
			Eigen::aligned_allocator<Eigen::Matrix4f> > correspondence_transformations;
	vector<Correspondences> clustered_corrs;
	GeometricConsistencyGrouping<PointXYZ, PointXYZ> geometric_consistency_grouping;
	geometric_consistency_grouping.setGCSize(param.geometric_consistency_GCsize);  //Sets the consensus set resolution
	geometric_consistency_grouping.setGCThreshold(param.geometric_consistency_GCThreshold);   //Sets the minimum cluster size
	geometric_consistency_grouping.setInputCloud(first_cloud);  
	geometric_consistency_grouping.setSceneCloud(second_cloud);
	geometric_consistency_grouping.setModelSceneCorrespondences(correspondences_ptr);
	geometric_consistency_grouping.recognize(correspondence_transformations, clustered_corrs);


	int largest_cluster_size = 0;
	int largest_cluster_index = -1;
	for (size_t i = 0u; i < clustered_corrs.size(); ++i)
	{
		if (clustered_corrs[i].size() >= largest_cluster_size)
		{
			largest_cluster_size = clustered_corrs[i].size();
			largest_cluster_index = i;
		}
	}


	if (largest_cluster_index < 0)
	{
		cout << "No valid matches found!" << endl;
		return -1;
	}
	else
	{
		// cout << "transformation:" << endl << correspondence_transformations[largest_cluster_index] << endl;
		transformation = correspondence_transformations[largest_cluster_index];
	}

	Correspondences& correspondences = clustered_corrs[largest_cluster_index];
	for (size_t i = 0; i < correspondences.size(); i++)
	{
		if (correspondences[i].index_query != correspondences[i].index_match)
		{
			cout << "correspondences[i].index_query != correspondences[i].index_match" << endl;
		}
		else filtered_matches.push_back(matches[correspondences[i].index_query]);
	}

	cout << filtered_matches.size() << " valid matches found!" << endl;

	return filtered_matches.size();
}



void SegMatch::trainRtrees(const Mat& features, const Mat& labels) 
{
	const unsigned int n_training_samples = features.rows;
	const unsigned int descriptors_dimension = features.cols;
	cout<< "Training RF with " << n_training_samples << " of dimension "
	<< descriptors_dimension << "."<<endl;

	float priors[] = {0.9, 0.1};

	// Random forest parameters.
	CvRTParams rtrees_params = CvRTParams(
										25,         		// max depth
										5, 					// min sample count
										0, 					// regression accuracy: N/A here
										false, 				// compute surrogate split, no missing data
										2, 					// max number of categories (use sub-optimal algorithm for larger numbers)
										priors, 			// the array of priors
										true,  				// calculate variable importance
										4,       			// number of variables randomly selected at node and used to find the best split(s).
										100,	 			// max number of trees in the forest
										0.0001f,				// forrest accuracy
										CV_TERMCRIT_EPS		// termination cirteria
										// CV_TERMCRIT_ITER
										);

	m_rtrees->train(features, CV_ROW_SAMPLE, labels, 
					cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), 
					rtrees_params);

	cout<<"Tree count: "<< m_rtrees->get_tree_count() << "."<<endl;

	Mat var_importance = m_rtrees->getVarImportance();
	if(var_importance.cols > 0)
	{
		double rt_imp_sum = sum(var_importance).val[0];
        printf("var#\timportance (in %%):\n");
        for(size_t i = 0; i < var_importance.cols; i++ )
            printf( "%-2lu\t%-4.1f\n", i,
            100.f*var_importance.at<float>(i)/rt_imp_sum);
    }
}	



void SegMatch::testRtrees(const Mat& features, const Mat& labels)
{
	const unsigned int n_samples = features.rows;
	const unsigned int descriptors_dimension = features.cols;
	cout<< "Testing the random forest with " << n_samples
	<< " samples of dimension " << descriptors_dimension << "."<<endl;


	if (n_samples > 0u) 
	{
		unsigned int tp = 0u, fp = 0u, tn = 0u, fn = 0u;
		for (unsigned int i = 0u; i < n_samples; ++i) 
		{
			double probability = m_rtrees->predict_prob(features.row(i));
			if (probability >= param.min_confidence) 
			{
				if (labels.at<float>(i, 0) == 1.0) ++tp;
				else ++fp;
			} 
			else 	
			{
				if (labels.at<float>(i, 0) == 0.0)++tn;
				else ++fn;
			}
		}
		displayPerformances(tp, tn, fp, fn);
	}
}


void SegMatch::saveRtrees(const std::string& filename)
{
  cout<< "Saving the classifier to: " << filename << "."<<endl;
  m_rtrees->save(filename.c_str());
}


void SegMatch::display(visualization::PCLVisualizer& p)
{
	int i = 0;
	printf("source segment size: %lu\n", m_source_segments.size());
	printf("target segment size: %lu\n", m_target_segments.size());
	for(auto seg = m_source_segments.begin(); seg != m_source_segments.end(); seg++)
	{   
		PointCloud<PointXYZI>::Ptr cloud(new PointCloud<PointXYZI>);

		pcl::copyPointCloud(seg->cloud, *cloud);

		for(size_t i = 0; i < cloud->size(); i++)
			cloud->at(i).z += 50.0;
		
		PointCloudColorHandlerCustom<PointXYZI> clr(cloud, rand ()%255, rand ()%255, rand ()%255);
		p.addPointCloud(cloud, clr, to_string(i));
		i++;
	}

	for(auto seg = m_target_segments.begin(); seg != m_target_segments.end(); seg++)
	{   
		PointCloudColorHandlerCustom<PointXYZI> clr(seg->cloud.makeShared(), rand ()%255, rand ()%255, rand ()%255);
		p.addPointCloud(seg->cloud.makeShared(), clr, to_string(i));
		i++;
	}
}


