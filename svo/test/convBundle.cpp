#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;

// Eigen::Quaterniond matrixIntoQuaternions(
//             float m11, float m12, float m13, // Make these function parameters!
//             float m21, float m22, float m23,
//             float m31, float m32, float m33
// )
// {

//     // Output quaternion
//     float w,x,y,z;
//     // Determine which of w,x,y, or z has the largest absolute value
//     float fourWSquaredMinus1 = m11 + m22 + m33;
//     float fourXSquaredMinus1 = m11 - m22 - m33;
//     float fourYSquaredMinus1 = m22 - m11 - m33;
//     float fourZSquaredMinus1 = m33 - m11 - m22;

//     int biggestIndex = 0;
//     float fourBiggestSquaredMinus1 = fourWSquaredMinus1;

//     if(fourXSquaredMinus1 > fourBiggestSquaredMinus1) {
//         fourBiggestSquaredMinus1 = fourXSquaredMinus1;
//         biggestIndex = 1;
//     }
//     if (fourYSquaredMinus1 > fourBiggestSquaredMinus1) {
//         fourBiggestSquaredMinus1 = fourYSquaredMinus1;
//         biggestIndex = 2;
//     }
//     if (fourZSquaredMinus1 > fourBiggestSquaredMinus1) {
//         fourBiggestSquaredMinus1 = fourZSquaredMinus1;
//         biggestIndex = 3;
//     }
//     // Per form square root and division
//     float biggestVal = sqrt (fourBiggestSquaredMinus1 + 1.0f ) * 0.5f;
//     float mult = 0.25f / biggestVal;

//     // Apply table to compute quaternion values
//     switch (biggestIndex) {
//         case 0:
//             w = biggestVal;
//             x = (m23 - m32) * mult;
//             y = (m31 - m13) * mult;
//             z = (m12 - m21) * mult;
//             break;
//         case 1:
//             x = biggestVal;
//             w = (m23 - m32) * mult;
//             y = (m12 + m21) * mult;
//             z = (m31 + m13) * mult;
//             break;
//         case 2:
//             y = biggestVal;
//             w = (m31 - m13) * mult;
//             x = (m12 + m21) * mult;
//             z = (m23 + m32) * mult;
//             break;
//         case 3:
//             z = biggestVal;
//             w = (m12 - m21) * mult;
//             x = (m31 + m13) * mult;
//             y = (m23 + m32) * mult;
//             break;
//         }

//     Eigen::Quaterniond q;
//     q.x() = x;
//     q.y() = y;
//     q.z() = z;
//     q.w() = w;
//     return q;
// } 

void convert(float scale) 
{
    std::fstream bundlefile;
    bundlefile.open("/home/worxli/Datasets/data/img/bundle/bundle.out",std::ios::in);

    ofstream outfile, outfile2, outfile3;
    outfile.open ("/home/worxli/Datasets/data/associate.txt");
    outfile2.open ("/home/worxli/Datasets/data/associate.ply");
    outfile3.open ("/home/worxli/Datasets/data/cameras_bundler.ply");

    outfile3 << "ply\n"
          << "format ascii 1.0\n"
          << "element face 0\n"
          << "property list uchar int vertex_indices\n"
          << "element vertex 206\n"
          << "property float x\n"
          << "property float y\n"
          << "property float z\n"
          << "end_header\n";

    if(!bundlefile.is_open())
    {
        fprintf(stderr,"\nERROR: Could not open File\n");
    } else {
        fprintf(stderr,"\nReading bundle File\n");

        std::string depthname; std::string rgbname;
        float focal, rad1, rad2, r11, r12, r13, r21, r22, r23, r31, r32, r33, translation1, translation2, translation3;
        float q1, q2, q3, q4;
        int cameras, points;

        std::string temp0("");
        getline(bundlefile,temp0); // # bundle file 0.3
        getline(bundlefile,temp0); // num_cameras, num_points

        std::stringstream stream0(temp0);

        stream0 >> cameras;
        stream0 >> points;

        int ind = 0;

        while(!bundlefile.eof() && ind < cameras)
        {
            ind = ind + 1;

            std::string temp1("");
            getline(bundlefile,temp1);
            std::stringstream stream1(temp1);
            stream1 >> focal; stream1 >> rad1; stream1 >> rad2;

            std::string temp2("");
            getline(bundlefile,temp2);
            std::stringstream stream2(temp2);
            stream2 >> r11; stream2 >> r12; stream2 >> r13;

            std::string temp3("");
            getline(bundlefile,temp3);
            std::stringstream stream3(temp3);
            stream3 >> r21; stream3 >> r22; stream3 >> r23;

            std::string temp4("");
            getline(bundlefile,temp4);
            std::stringstream stream4(temp4);
            stream4 >> r31; stream4 >> r32; stream4 >> r33;

            std::string temp5("");
            getline(bundlefile,temp5);
            std::stringstream stream5(temp5);
            stream5 >> translation1; stream5 >> translation2; stream5 >> translation3;

            // Eigen::Quaterniond q = matrixIntoQuaternions(r11,r12,r13,
            //                                     r21,r22,r23,
            //                                     r31,r32,r33);

            // Vec4f quat = matrixIntoQuaternions(r11,r21,r31,
                                                // r12,r22,r32,
                                                // r13,r23,r33);

            Eigen::Matrix3f m;
            m << r11, r12, r13,
                 r21, r22, r23,
                 r31, r32, r33;
            // m << r11, r21, r31,
            //      r12, r22, r32,
            //      r13, r23, r33;
            // std::cout << m << endl;
            Eigen::Quaternionf q1(m);

            outfile << translation1 * scale << " "
                << translation2 * scale << " "
                << translation3 * scale << " "
                << q1.z()  << " "
                << q1.w()  << " "
                << q1.x()  << " "
                << q1.y()  << " "  
                // << q1.z()  << " " 
                // << q1.w()  << " " 
                << "depth/mapped" << ind << ".png "
                << "img/color" << ind << ".png\n";

            outfile3 << translation1 * scale << " " << translation2 * scale << " " << translation3 * scale << "\n" << endl;


        }

        outfile3.close();
        outfile.close();
        cout << "wrote associate file" << endl;
        float x,y,z;

        outfile2 << "ply\n"
          << "format ascii 1.0\n"
          << "element face 0\n"
          << "property list uchar int vertex_indices\n"
          << "element vertex 3527\n"
          << "property float x\n"
          << "property float y\n"
          << "property float z\n"
          << "end_header\n";

        while(!bundlefile.eof())
        {
            std::string temp("");
            std::string temp5("");
            std::string temp6("");
            getline(bundlefile,temp);
            std::stringstream stream(temp);
            stream >> x; stream >> y; stream >> z;
            getline(bundlefile,temp5);
            std::stringstream stream5(temp5);
            getline(bundlefile,temp6);
            std::stringstream stream6(temp6);
            // cout << x << " " << y << " " << z << endl;
            outfile2 << x << " " << y << " " << z << "\n";
        }

        outfile2.close();
        cout << "wrote ply" << endl;
    }
}

int main(int argc, char** argv)
{

    if ( argc != 2 ) {
        convert(1);
    } else {
        convert(std::stof(argv[1]));
    }

    return 0;
}