#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Butterworth.hpp"

#define MAXBUFSIZE  ((int) 1e6)

Eigen::MatrixXd readMatrix(const char *filename) {
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
	std::ifstream infile;
    infile.open(filename);
    while (! infile.eof()) {
		std::string line;
        getline(infile, line);

        int temp_cols = 0;
		std::stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
	}

    infile.close();

    rows--;

    // Populate matrix with numbers.
	Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    return result;
}
   
void writeMatrix(const Eigen::MatrixXd& src, const char* path) {
	std::ofstream fileout(path, std::ios::out | std::ios::trunc);  
     if(fileout.is_open()) {   
       fileout << src << "\n";
       fileout.close();
     } else {
   	  std::cerr << "Error writing the file" << std::endl;
     }
}




int main(int argc, char** argv) {

	std::string filename = "/home/ltonin/test_eog1.txt";
	std::string fileout = "/home/ltonin/test_eog1_filtered.txt";

	ros::init(argc, argv, "test_butterworth");

	Eigen::MatrixXd input = readMatrix(filename.c_str());

	unsigned int nsamples  = input.rows();
	unsigned int nchannels = input.cols();

	rosneuro::NeuroData<double>  in(nsamples, nchannels, "EEG");
	rosneuro::NeuroData<double> out(nsamples, nchannels, "EEG");
	rosneuro::NeuroData<double> out2(nsamples, nchannels, "EEG");
	rosneuro::NeuroData<double> out3(nsamples, nchannels, "EEG");
	rosneuro::NeuroData<double> out4(nsamples, nchannels, "EEG");
	
	rosneuro::Filter<double>* butter_lp = new rosneuro::Butterworth<double>();
	if(butter_lp->configure("ButterworthLowPass") == false) {
		ROS_ERROR("Butterworth filter configuration failed");
		return false;
	}
	ROS_INFO("Butterworth filter configuration succeeded");
	
	rosneuro::Filter<double>* butter_hp = new rosneuro::Butterworth<double>();
	if(butter_hp->configure("ButterworthHighPass") == false) {
		ROS_ERROR("Butterworth filter configuration failed");
		return false;
	}
	ROS_INFO("Butterworth filter configuration succeeded");

			
	std::copy(input.data(), input.data() + input.size(), in.data());

	if(butter_lp->apply(in, out) == false) {
		std::cout<<"Butterworth filter failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
	if(butter_lp->apply(out, out2) == false) {
		std::cout<<"Butterworth filter failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
	if(butter_hp->apply(in, out3) == false) {
		std::cout<<"Butterworth filter failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
	if(butter_hp->apply(out3, out4) == false) {
		std::cout<<"Butterworth filter failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
	
		
	std::cout<<"Butterworth filter succesfully applied"<<std::endl;
	Eigen::Map<Eigen::MatrixXd> eout(out4.data(), nsamples, nchannels);


	writeMatrix(eout, fileout.c_str());
	ros::shutdown();

	
	return 0;

}
