#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;

void c2w(float transscale, float rotscale) 
{

    Eigen::Matrix3d k;
    k << 584.6, 0, 319.5, 0, 580.9, 239, 0, 0, 1;
    int depthscale = 1;

    int counter = 0;
    std::fstream associationfile;
    associationfile.open("/home/worxli/Datasets/data/associate.txt",std::ios::in);

    ofstream outfile;
    outfile.open ("/home/worxli/Datasets/data/depth.ply");

    outfile << "ply\n"
          << "format ascii 1.0\n"
          << "element face 0\n"
          << "property list uchar int vertex_indices\n"
          << "element vertex 57600\n"
          << "property float x\n"
          << "property float y\n"
          << "property float z\n"
          << "end_header\n";

    if(!associationfile.is_open())
    {
        fprintf(stderr,"\nERROR: Could not open File\n");
    } else {
        fprintf(stderr,"\nReading Association unscaled File\n");

        std::string depthname; std::string rgbname;
        float q1, q2, q3, q4, translation1, translation2, translation3;

        while(!associationfile.eof())
        {
            std::string temp("");
            getline(associationfile,temp);
            temp = "";
            getline(associationfile,temp);
            temp = "";
            getline(associationfile,temp);
            temp = "";
            getline(associationfile,temp);
            temp = "";
            getline(associationfile,temp);
            temp = "";
            getline(associationfile,temp);

            std::stringstream stream(temp);

            stream >> translation1; stream >> translation2; stream >> translation3;
            stream >> q1; stream >> q2; stream >> q3; stream >> q4;
            stream >> depthname;
            stream >> rgbname;

            std::stringstream name;
            name << "/home/worxli/Datasets/data/" << depthname;
            Mat img = cv::imread(name.str(), -1);
            

            Eigen::Matrix3d r = Eigen::Quaterniond(q4,q1,q2,q3).toRotationMatrix() * rotscale;
            Eigen::Vector3d t = Eigen::Vector3d(translation1 * transscale, translation2 * transscale, translation3 * transscale);
            // Eigen::Vector3d t = Eigen::Vector3d(translation1, translation2, translation3);

            //with U = [D*u; D*v; D]'  // coordinate in pixels (u,v), depth D, intrinsic K, extrinsics [R T], 3D point X
            //so X = inv(R) * ( inv(K) * U - T )

            Eigen::Vector3d pos = ((-1) * r.transpose()) * t;
            // Eigen::Vector3d pos = t;
            outfile << pos[0] << " " << pos[1] << " " << pos[2] << endl;

            // Eigen::Vector3d vec = r.transpose()) * t;

            cout << "image: " << name.str() << endl;

            for (int y = 0; y < img.rows; y=y+4)
             {
                for (int x = 0; x < img.cols; x=x+4)
                 {
                    uint16_t realDepth2 = img.at<uint16_t>(y, x);
                    double realDepth = (double)realDepth2/5000;

                    // if(realDepth>100)
                    // {
                        // cout << x << " " << realDepth << " " << (double)x/realDepth << endl;
                        // Eigen::Vector3d vec = Eigen::Vector3d((double)x/realDepth, (double)y/realDepth, depth);
                        Eigen::Vector3d u = Eigen::Vector3d((double)x*realDepth, (double)y*realDepth, realDepth);

                        Eigen::Vector3d vec = r.inverse() * ( k.inverse() * u - t);
                        // Eigen::Vector3d vec = k.inverse() * u;

                        // if(abs(vec[0]) < 5 && abs(vec[1]) < 5 && abs(vec[2]) < 5)
                        // outfile << vec[0]/depthscale << " " << vec[1]/depthscale << " " << vec[2]/depthscale << endl;
                        outfile << vec[0] << " " << vec[1] << " " << vec[2] << endl;
                        // cout << x << " " << y << " " << realDepth << " " << depth << endl;
                        // cout << vec[0] << " " << vec[1] << " " << vec[2] << endl;
                    // }
                }
            }

            if(++counter>2)
                break;
        }

        outfile.close();
        cout << "wrote ply depth file" << endl;
  }
}

int main(int argc, char** argv)
{

    if ( argc == 3 ) {
        c2w(std::stof(argv[1]),std::stof(argv[2]));
    } else {
        c2w(10000, 2);
    }

    return 0;
}